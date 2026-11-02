//! `.ron` and JSON descriptors: one serde schema, two syntaxes
//! (contracts/descriptor-ron.md; research R8).
//!
//! Model data only: every struct denies unknown fields, so a group, a port
//! or a baud rate does not build (FR-005 (SRS FR-6.2), FR-006 (SRS FR-6.1)).
//! Faults found after parsing have no position.

use serde::{Deserialize, Deserializer};

use crate::model::{Actuator, Joint, Kind, Limit, Link, Model, ModelError, Pos, Reader};

/// The `.ron` reader.
pub(crate) struct Ron;

/// The JSON reader.
pub(crate) struct Json;

impl Reader for Ron {
    fn read(text: &str, file: &str) -> Result<Model, ModelError> {
        let robot: Robot = ron::from_str(text).map_err(|e| {
            let at = (line_col(e.span.start.line), line_col(e.span.start.col));
            ModelError::syntax(file, at, e.code.to_string())
        })?;
        robot.model(file)
    }
}

impl Reader for Json {
    fn read(text: &str, file: &str) -> Result<Model, ModelError> {
        let robot: Robot = serde_json::from_str(text).map_err(|e| {
            let at: Pos = (line_col(e.line()), line_col(e.column()));
            let text = e.to_string();
            let suffix = format!(" at line {} column {}", e.line(), e.column());
            let reason = text.strip_suffix(&suffix).unwrap_or(&text);
            ModelError::syntax(file, at, reason)
        })?;
        robot.model(file)
    }
}

/// A parser's line or column as a [`Pos`] part.
fn line_col(n: usize) -> u32 {
    u32::try_from(n).unwrap_or(u32::MAX)
}

/// A whole descriptor.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Robot {
    /// Required by the schema; the model does not use it.
    #[allow(dead_code)]
    name: String,
    /// Every link.
    links: Vec<RobotLink>,
    /// Every joint; the non-fixed ones are the actuators, in this order.
    joints: Vec<RobotJoint>,
}

/// One link.
#[derive(Deserialize)]
#[serde(deny_unknown_fields, rename = "Link")]
struct RobotLink {
    /// The link's name.
    name: String,
    /// kg; 0 when left out, as a URDF link without `<inertial>`.
    #[serde(default)]
    mass: f64,
    /// Mass center, m, link frame; the link origin when left out.
    #[serde(default)]
    com: [f64; 3],
}

/// One joint.
#[derive(Deserialize)]
#[serde(deny_unknown_fields, rename = "Joint")]
struct RobotJoint {
    /// The joint's name.
    name: String,
    /// What the joint does.
    kind: JointKind,
    /// The parent link's name.
    parent: String,
    /// The child link's name.
    child: String,
    /// Required for every non-fixed joint.
    limit: Option<RobotLimit>,
}

/// The joint kinds of the schema.
#[derive(Clone, Copy, Deserialize, PartialEq)]
#[serde(rename = "Kind")]
enum JointKind {
    /// Bounded rotation, rad.
    Revolute,
    /// Unbounded rotation, rad.
    Continuous,
    /// Translation, m.
    Prismatic,
    /// No actuator.
    Fixed,
}

/// A joint's limit; `lower` and `upper` are written bare, not in `Some`.
#[derive(Deserialize)]
#[serde(deny_unknown_fields, rename = "Limit")]
struct RobotLimit {
    /// Lowest position; required for `Revolute` and `Prismatic`, refused
    /// for `Continuous`.
    #[serde(default, deserialize_with = "bare")]
    lower: Option<f64>,
    /// Highest position; as `lower`.
    #[serde(default, deserialize_with = "bare")]
    upper: Option<f64>,
    /// Top speed.
    velocity: f64,
    /// Top effort.
    effort: f64,
}

/// A number that is there, as `Some`.
fn bare<'de, D: Deserializer<'de>>(d: D) -> Result<Option<f64>, D::Error> {
    f64::deserialize(d).map(Some)
}

impl Robot {
    /// The model this descriptor writes, steps 3 and 4 of data-model.md.
    fn model(self, file: &str) -> Result<Model, ModelError> {
        let mut model = Model {
            links: self
                .links
                .into_iter()
                .map(|l| Link {
                    name: l.name,
                    mass: l.mass,
                    com: l.com,
                    pos: None,
                })
                .collect(),
            ..Model::default()
        };
        for j in self.joints {
            let kind = match j.kind {
                JointKind::Revolute => Some(Kind::Revolute),
                JointKind::Continuous => Some(Kind::Continuous),
                JointKind::Prismatic => Some(Kind::Prismatic),
                JointKind::Fixed => None,
            };
            if let Some(kind) = kind {
                let limit = j.limit.as_ref().map(|l| Limit {
                    lower: l.lower,
                    upper: l.upper,
                    velocity: Some(l.velocity),
                    effort: Some(l.effort),
                });
                if kind == Kind::Continuous
                    && let Some(field) = limit.and_then(|l| {
                        (l.lower.is_some().then_some("lower")).or(l.upper.map(|_| "upper"))
                    })
                {
                    let reason = format!("a Continuous joint takes no {field}");
                    return Err(ModelError::element(
                        file,
                        None,
                        format!("joint {}", j.name),
                        reason,
                    ));
                }
                model.actuators.push(Actuator {
                    name: j.name.clone(),
                    kind,
                    limit,
                    pos: None,
                });
            }
            model.joints.push(Joint {
                name: j.name,
                parent: j.parent,
                child: j.child,
                pos: None,
            });
        }
        Ok(model)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::urdf::{Urdf, tests::FOUR};

    /// kmr_api/tests/fixtures/four.ron, inline.
    const FOUR_RON: &str = r#"Robot(
    name: "four",
    links: [
        (name: "base", mass: 0.5, com: (0.0, 0.0, 0.02)),
        (name: "l1", mass: 0.35, com: (0.0, 0.0, 0.05)),
        (name: "l2", mass: 0.3, com: (0.0, 0.0, 0.04)),
        (name: "l3", mass: 0.2, com: (0.0, 0.0, 0.03)),
        (name: "l4", mass: 0.1, com: (0.0, 0.0, 0.02)),
        (name: "tool", mass: 0.05, com: (0.0, 0.0, 0.01)),
    ],
    joints: [
        (name: "j1", kind: Revolute, parent: "base", child: "l1",
         limit: Some((lower: -1.57, upper: 1.57, velocity: 3.0, effort: 1.5))),
        (name: "j2", kind: Continuous, parent: "l1", child: "l2",
         limit: Some((velocity: 3.0, effort: 1.5))),
        (name: "j3", kind: Prismatic, parent: "l2", child: "l3",
         limit: Some((lower: 0.0, upper: 0.1, velocity: 0.05, effort: 20.0))),
        (name: "j4", kind: Revolute, parent: "l3", child: "l4",
         limit: Some((lower: -3.14, upper: 3.14, velocity: 4.0, effort: 2.0))),
        (name: "mount", kind: Fixed, parent: "l4", child: "tool"),
    ],
)
"#;

    /// kmr_api/tests/fixtures/four.json, inline.
    const FOUR_JSON: &str = r#"{
  "name": "four",
  "links": [
    { "name": "base", "mass": 0.5, "com": [0.0, 0.0, 0.02] },
    { "name": "l1", "mass": 0.35, "com": [0.0, 0.0, 0.05] },
    { "name": "l2", "mass": 0.3, "com": [0.0, 0.0, 0.04] },
    { "name": "l3", "mass": 0.2, "com": [0.0, 0.0, 0.03] },
    { "name": "l4", "mass": 0.1, "com": [0.0, 0.0, 0.02] },
    { "name": "tool", "mass": 0.05, "com": [0.0, 0.0, 0.01] }
  ],
  "joints": [
    { "name": "j1", "kind": "Revolute", "parent": "base", "child": "l1",
      "limit": { "lower": -1.57, "upper": 1.57, "velocity": 3.0, "effort": 1.5 } },
    { "name": "j2", "kind": "Continuous", "parent": "l1", "child": "l2",
      "limit": { "velocity": 3.0, "effort": 1.5 } },
    { "name": "j3", "kind": "Prismatic", "parent": "l2", "child": "l3",
      "limit": { "lower": 0.0, "upper": 0.1, "velocity": 0.05, "effort": 20.0 } },
    { "name": "j4", "kind": "Revolute", "parent": "l3", "child": "l4",
      "limit": { "lower": -3.14, "upper": 3.14, "velocity": 4.0, "effort": 2.0 } },
    { "name": "mount", "kind": "Fixed", "parent": "l4", "child": "tool" }
  ]
}
"#;

    /// `model` with every position dropped.
    fn unplaced(mut model: Model) -> Model {
        model.actuators.iter_mut().for_each(|a| a.pos = None);
        model.joints.iter_mut().for_each(|j| j.pos = None);
        model.links.iter_mut().for_each(|l| l.pos = None);
        model
    }

    fn ron(text: &str) -> Result<Model, String> {
        Ron::read(text, "r.ron").map_err(|e| e.message())
    }

    fn json(text: &str) -> Result<Model, String> {
        Json::read(text, "r.json").map_err(|e| e.message())
    }

    #[test]
    fn ron_and_json_give_the_model_of_the_urdf() {
        let urdf = Urdf::read(FOUR, "r.urdf")
            .map(unplaced)
            .map_err(|e| e.message());
        assert!(urdf.is_ok(), "{urdf:?}");
        assert_eq!(ron(FOUR_RON), urdf);
        assert_eq!(json(FOUR_JSON), urdf);
    }

    #[test]
    fn an_unknown_field_has_line_and_column() {
        let text = FOUR_RON.replace("    joints:", "    groups: [],\n    joints:");
        let err = ron(&text);
        assert!(
            err.as_ref()
                .is_err_and(|e| e.starts_with("kmr: r.ron:11:") && e.contains("groups")),
            "{err:?}"
        );
        let text = FOUR_JSON.replace("\"name\": \"four\",", "\"name\": \"four\", \"groups\": [],");
        let err = json(&text);
        assert!(
            err.as_ref().is_err_and(
                |e| e.starts_with("kmr: r.json:2:") && e.contains("unknown field `groups`")
            ),
            "{err:?}"
        );
    }

    #[test]
    fn a_hardware_field_on_a_joint_is_unknown() {
        let text = FOUR_RON.replace("child: \"tool\"", "child: \"tool\", baud: 1000000");
        let err = ron(&text);
        assert!(
            err.as_ref().is_err_and(
                |e| e.starts_with("kmr: r.ron:20:") && e.contains("named `baud` in `Joint`")
            ),
            "{err:?}"
        );
    }

    #[test]
    fn a_syntax_error_has_line_and_column() {
        let err = ron("Robot(\n  name: \"t\",\n  links: [\n)");
        assert!(
            err.as_ref().is_err_and(|e| e.starts_with("kmr: r.ron:4:")),
            "{err:?}"
        );
        let err = json("{\n  \"name\": \"t\",\n}");
        assert_eq!(err, Err("kmr: r.json:3:1: trailing comma".to_owned()));
    }

    #[test]
    fn a_continuous_joint_takes_no_position_limit() {
        let text = FOUR_RON.replace(
            "limit: Some((velocity: 3.0,",
            "limit: Some((lower: -1.0, velocity: 3.0,",
        );
        assert_eq!(
            ron(&text),
            Err("kmr: r.ron: joint j2: a Continuous joint takes no lower".to_owned())
        );
    }

    #[test]
    fn fixed_joints_get_no_index() {
        let names =
            ron(FOUR_RON).map(|m| m.actuators.into_iter().map(|a| a.name).collect::<Vec<_>>());
        assert_eq!(
            names,
            Ok(["j1", "j2", "j3", "j4"].map(String::from).to_vec())
        );
    }

    #[test]
    fn a_link_without_mass_has_none_at_its_origin() {
        let text = FOUR_RON.replace(
            "(name: \"base\", mass: 0.5, com: (0.0, 0.0, 0.02))",
            "(name: \"base\")",
        );
        let base = ron(&text).map(|m| (m.links[0].mass, m.links[0].com));
        assert_eq!(base, Ok((0.0, [0.0; 3])));
    }
}
