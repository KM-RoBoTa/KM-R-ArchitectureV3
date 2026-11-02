//! The build-time robot model and its checks (data-model.md).
//!
//! A [`Reader`] turns a descriptor into a [`Model`], as written; [`validate`]
//! checks it and gives the [`Robot`] that `emit` writes out.

use std::collections::HashMap;
use std::path::PathBuf;

/// A descriptor syntax, picked by the file extension.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Format {
    /// `.urdf`
    Urdf,
    /// `.ron`
    Ron,
    /// `.json`
    Json,
}

/// One robot descriptor: where it is, its syntax and its text.
pub(crate) struct Descriptor {
    /// The file, absolute, or the shown path of a preset.
    pub(crate) path: PathBuf,
    /// The syntax of `text`.
    pub(crate) format: Format,
    /// The whole file.
    pub(crate) text: String,
}

/// A place in a descriptor: line and column, both from 1.
pub(crate) type Pos = (u32, u32);

/// The joint kinds that drive one actuator.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Kind {
    /// Bounded rotation, in rad.
    Revolute,
    /// Unbounded rotation, in rad: no position limit.
    Continuous,
    /// Translation, in m as written, in mm once checked (SRS FR-2.2).
    Prismatic,
}

/// A joint's `<limit>`, each value as written.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct Limit {
    /// Lowest position.
    pub(crate) lower: Option<f64>,
    /// Highest position.
    pub(crate) upper: Option<f64>,
    /// Top speed.
    pub(crate) velocity: Option<f64>,
    /// Top effort.
    pub(crate) effort: Option<f64>,
}

/// One actuated joint, as read.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Actuator {
    /// The joint's name.
    pub(crate) name: String,
    /// What the joint does.
    pub(crate) kind: Kind,
    /// `None` when the joint has no `<limit>`.
    pub(crate) limit: Option<Limit>,
    /// Where the joint is written, when the reader knows.
    pub(crate) pos: Option<Pos>,
}

/// One joint of any kind, fixed included: an edge of the link tree.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Joint {
    /// The joint's name.
    pub(crate) name: String,
    /// The parent link's name.
    pub(crate) parent: String,
    /// The child link's name.
    pub(crate) child: String,
    /// Where the joint is written, when the reader knows.
    pub(crate) pos: Option<Pos>,
}

/// One link, as read.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Link {
    /// The link's name.
    pub(crate) name: String,
    /// Mass, kg.
    pub(crate) mass: f64,
    /// Mass center in the link frame, m.
    pub(crate) com: [f64; 3],
    /// Where the link is written, when the reader knows.
    pub(crate) pos: Option<Pos>,
}

/// A descriptor as read, before the checks.
#[derive(Clone, Debug, Default, PartialEq)]
pub(crate) struct Model {
    /// The actuated joints, in file order: index `i` is actuator `i`.
    pub(crate) actuators: Vec<Actuator>,
    /// Every joint, in file order.
    pub(crate) joints: Vec<Joint>,
    /// Every link, in file order.
    pub(crate) links: Vec<Link>,
}

/// One actuator after the checks, in SRS FR-2.2 units: rad, rad/s, N·m,
/// or mm, mm/s, N for a prismatic joint.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct CheckedActuator {
    /// The joint's name.
    pub(crate) name: String,
    /// What the joint does.
    pub(crate) kind: Kind,
    /// Lowest position; `0.0` for a continuous joint.
    pub(crate) lower: f32,
    /// Highest position; `0.0` for a continuous joint.
    pub(crate) upper: f32,
    /// Top speed.
    pub(crate) speed: f32,
    /// Top effort.
    pub(crate) effort: f32,
}

/// One link after the checks.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct CheckedLink {
    /// The link's name.
    pub(crate) name: String,
    /// Index of the parent link; `None` for the root.
    pub(crate) parent: Option<u16>,
    /// Mass, kg.
    pub(crate) mass: f32,
    /// Mass center in the link frame, m.
    pub(crate) com: [f32; 3],
}

/// A checked robot: what `emit` writes out.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Robot {
    /// In index order.
    pub(crate) actuators: Vec<CheckedActuator>,
    /// In file order.
    pub(crate) links: Vec<CheckedLink>,
}

/// Why a descriptor does not build (contracts/robot-macro.md § Errors).
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ModelError {
    /// The descriptor, as shown to the user.
    pub(crate) file: String,
    /// The line of the fault, when known.
    pub(crate) line: Option<u32>,
    /// The column, shown only for a fault the parser found.
    pub(crate) col: Option<u32>,
    /// The joint, link or transmission at fault, e.g. `joint j1`.
    pub(crate) element: Option<String>,
    /// What is wrong.
    pub(crate) reason: String,
    /// The SRS reference, when the descriptor asks for something out of scope.
    pub(crate) marker: Option<&'static str>,
}

impl ModelError {
    /// A fault of the whole file, at no line.
    pub(crate) fn file(file: &str, reason: impl Into<String>) -> Self {
        Self {
            file: file.to_owned(),
            line: None,
            col: None,
            element: None,
            reason: reason.into(),
            marker: None,
        }
    }

    /// A syntax fault at a line and column.
    pub(crate) fn syntax(file: &str, (line, col): Pos, reason: impl Into<String>) -> Self {
        Self {
            line: Some(line),
            col: Some(col),
            ..Self::file(file, reason)
        }
    }

    /// A fault of one element, at its line when known.
    pub(crate) fn element(
        file: &str,
        pos: Option<Pos>,
        element: impl Into<String>,
        reason: impl Into<String>,
    ) -> Self {
        Self {
            line: pos.map(|(line, _)| line),
            element: Some(element.into()),
            ..Self::file(file, reason)
        }
    }

    /// The same fault, with an SRS marker.
    pub(crate) fn marked(self, marker: &'static str) -> Self {
        Self {
            marker: Some(marker),
            ..self
        }
    }

    /// The compile error text: `kmr: <file>[:<line>[:<col>]]: [<element>: ]<reason>[: <marker>]`.
    pub(crate) fn message(&self) -> String {
        let mut out = format!("kmr: {}", self.file);
        if let Some(line) = self.line {
            out += &format!(":{line}");
            if let Some(col) = self.col {
                out += &format!(":{col}");
            }
        }
        let parts = [
            self.element.as_deref(),
            Some(self.reason.as_str()).filter(|r| !r.is_empty()),
            self.marker,
        ];
        for part in parts.into_iter().flatten() {
            out += ": ";
            out += part;
        }
        out
    }
}

/// The marker of a joint that needs more than one actuator (FR-009).
pub(crate) const MULTI_ACTUATOR: &str = "SRS index space: multi-actuator joint";

/// The marker of a descriptor holding several robots (FR-011).
pub(crate) const MULTI_ROBOT: &str = "SRS FR-6.5: multi-robot descriptor";

/// The marker of a joint with more than one degree of freedom (FR-010).
pub(crate) const NON_1_DOF: &str = "roadmap: non-1-DOF joints";

/// Reads one descriptor syntax into a [`Model`] (FR-025 (SRS FR-11.3)).
///
/// It runs validation steps 2 to 5 of data-model.md; [`validate`] runs the
/// rest, the same for every syntax.
pub(crate) trait Reader {
    /// The model `text` describes; `file` is the path shown in errors.
    fn read(text: &str, file: &str) -> Result<Model, ModelError>;
}

/// Validation steps 6 to 10 of data-model.md, in order; the first failure
/// stops.
pub(crate) fn validate(model: Model, file: &str) -> Result<Robot, ModelError> {
    unique_names(&model, file)?;
    for a in &model.actuators {
        check_limit(a, file)?;
    }
    let actuators = model
        .actuators
        .iter()
        .map(|a| actuator_units(a, file))
        .collect::<Result<Vec<_>, _>>()?;
    let bodies = model
        .links
        .iter()
        .map(|l| link_units(l, file))
        .collect::<Result<Vec<_>, _>>()?;
    let parents = tree(&model, file)?;
    if actuators.is_empty() {
        return Err(ModelError::file(file, "no actuated joint"));
    }
    let links = model
        .links
        .iter()
        .zip(bodies)
        .zip(parents)
        .map(|((l, (mass, com)), parent)| {
            let parent = parent.map(u16::try_from).transpose().map_err(|_| {
                ModelError::element(
                    file,
                    l.pos,
                    format!("link {}", l.name),
                    "parent index does not fit u16",
                )
            })?;
            Ok(CheckedLink {
                name: l.name.clone(),
                parent,
                mass,
                com,
            })
        })
        .collect::<Result<_, ModelError>>()?;
    Ok(Robot { actuators, links })
}

/// Step 6: joint names and link names are unique.
fn unique_names(model: &Model, file: &str) -> Result<(), ModelError> {
    let joints = model.joints.iter().map(|j| (j.name.as_str(), j.pos));
    let links = model.links.iter().map(|l| (l.name.as_str(), l.pos));
    first_duplicate("joint", joints, file)?;
    first_duplicate("link", links, file)
}

/// The second use of a name among `items`, as an error.
fn first_duplicate<'a>(
    what: &str,
    items: impl Iterator<Item = (&'a str, Option<Pos>)>,
    file: &str,
) -> Result<(), ModelError> {
    let mut seen: HashMap<&str, Option<Pos>> = HashMap::new();
    for (name, pos) in items {
        if let Some(first) = seen.get(name) {
            let reason = match first {
                Some((line, _)) => format!("duplicate {what} {name} (first at line {line})"),
                None => format!("duplicate {what} {name}"),
            };
            return Err(ModelError {
                line: pos.map(|(l, _)| l),
                ..ModelError::file(file, reason)
            });
        }
        seen.insert(name, pos);
    }
    Ok(())
}

/// Step 7: the limit a joint of this kind needs is there and in range.
fn check_limit(a: &Actuator, file: &str) -> Result<(), ModelError> {
    let fault =
        |reason: &str| ModelError::element(file, a.pos, format!("joint {}", a.name), reason);
    let Some(limit) = a.limit else {
        return Err(fault("no <limit>"));
    };
    let bounded = a.kind != Kind::Continuous;
    let needed = [
        ("lower", limit.lower.is_some() || !bounded),
        ("upper", limit.upper.is_some() || !bounded),
        ("velocity", limit.velocity.is_some()),
        ("effort", limit.effort.is_some()),
    ];
    if let Some((field, _)) = needed.iter().find(|(_, there)| !there) {
        return Err(fault(&format!("<limit> has no {field}")));
    }
    if let (true, Some(lower), Some(upper)) = (bounded, limit.lower, limit.upper)
        && lower > upper
    {
        return Err(fault("lower > upper"));
    }
    if limit.velocity.is_some_and(|v| v < 0.0) {
        return Err(fault("velocity < 0"));
    }
    if limit.effort.is_some_and(|e| e < 0.0) {
        return Err(fault("effort < 0"));
    }
    Ok(())
}

/// Step 8 for an actuator: SRS FR-2.2 units (prismatic m to mm), every
/// value finite and within `f32`. Runs after step 7.
fn actuator_units(a: &Actuator, file: &str) -> Result<CheckedActuator, ModelError> {
    let limit = a.limit.unwrap_or_default();
    let scale = if a.kind == Kind::Prismatic {
        1000.0
    } else {
        1.0
    };
    let fit = |field: &str, v: Option<f64>, scale: f64| {
        to_f32(v.unwrap_or(0.0) * scale).ok_or_else(|| {
            ModelError::element(
                file,
                a.pos,
                format!("joint {}", a.name),
                format!("{field} does not fit f32"),
            )
        })
    };
    let (lower, upper) = match a.kind {
        Kind::Continuous => (0.0, 0.0),
        _ => (
            fit("lower", limit.lower, scale)?,
            fit("upper", limit.upper, scale)?,
        ),
    };
    Ok(CheckedActuator {
        name: a.name.clone(),
        kind: a.kind,
        lower,
        upper,
        speed: fit("velocity", limit.velocity, scale)?,
        effort: fit("effort", limit.effort, 1.0)?,
    })
}

/// `v` as an `f32`, when finite and in range.
pub(crate) fn to_f32(v: f64) -> Option<f32> {
    (v.is_finite() && v.abs() <= f64::from(f32::MAX)).then_some(v as f32)
}

/// Step 8 for a link: mass and mass center finite and within `f32`.
fn link_units(l: &Link, file: &str) -> Result<(f32, [f32; 3]), ModelError> {
    let fit = |field: &str, v: f64| {
        to_f32(v).ok_or_else(|| {
            ModelError::element(
                file,
                l.pos,
                format!("link {}", l.name),
                format!("{field} does not fit f32"),
            )
        })
    };
    Ok((
        fit("mass", l.mass)?,
        [
            fit("com x", l.com[0])?,
            fit("com y", l.com[1])?,
            fit("com z", l.com[2])?,
        ],
    ))
}

/// Step 9: the links form one tree and every joint names known links; the
/// parent index of each link, `None` for the root.
fn tree(model: &Model, file: &str) -> Result<Vec<Option<usize>>, ModelError> {
    let index: HashMap<&str, usize> = model
        .links
        .iter()
        .enumerate()
        .map(|(i, l)| (l.name.as_str(), i))
        .collect();
    let mut parent: Vec<Option<usize>> = vec![None; model.links.len()];
    let mut parent_joint: Vec<Option<&str>> = vec![None; model.links.len()];
    for j in &model.joints {
        let fault =
            |reason: String| ModelError::element(file, j.pos, format!("joint {}", j.name), reason);
        let known = |role: &str, name: &str| {
            index
                .get(name)
                .copied()
                .ok_or_else(|| fault(format!("{role} link {name} is not defined")))
        };
        let p = known("parent", &j.parent)?;
        let c = known("child", &j.child)?;
        if let Some(other) = parent_joint[c] {
            return Err(fault(format!(
                "link {} is already the child of joint {other}",
                j.child
            )));
        }
        parent[c] = Some(p);
        parent_joint[c] = Some(&j.name);
    }
    let roots: Vec<&str> = model
        .links
        .iter()
        .zip(&parent)
        .filter(|(_, p)| p.is_none())
        .map(|(l, _)| l.name.as_str())
        .collect();
    if roots.len() > 1 {
        let reason = format!("{} root links ({})", roots.len(), roots.join(", "));
        return Err(ModelError::file(file, reason).marked(MULTI_ROBOT));
    }
    for (i, link) in model.links.iter().enumerate() {
        let mut at = i;
        for _ in 0..model.links.len() {
            match parent[at] {
                Some(p) => at = p,
                None => break,
            }
        }
        if parent[at].is_some() {
            return Err(ModelError::element(
                file,
                link.pos,
                format!("link {}", link.name),
                "is in a loop of joints",
            ));
        }
    }
    Ok(parent)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::urdf::Urdf;

    /// A joint element with an optional `<limit>` and extra children.
    fn joint(
        name: &str,
        kind: &str,
        parent: &str,
        child: &str,
        limit: &str,
        extra: &str,
    ) -> String {
        format!(
            "  <joint name=\"{name}\" type=\"{kind}\"><parent link=\"{parent}\"/><child link=\"{child}\"/>{limit}{extra}</joint>\n"
        )
    }

    /// A full limit.
    const LIMIT: &str = "<limit lower=\"-1\" upper=\"1\" velocity=\"2\" effort=\"3\"/>";

    /// A `<robot>` with links `base`, `l1`, `l2` and `body` between them.
    fn robot(body: &str) -> String {
        format!(
            "<robot name=\"t\">\n  <link name=\"base\"/><link name=\"l1\"/><link name=\"l2\"/>\n{body}</robot>\n"
        )
    }

    /// The two-joint chain base, l1, l2, with `j1` and `j2` revolute.
    fn two(j1_limit: &str, j2_limit: &str) -> String {
        robot(
            &(joint("j1", "revolute", "base", "l1", j1_limit, "")
                + &joint("j2", "revolute", "l1", "l2", j2_limit, "")),
        )
    }

    fn check(text: &str) -> Result<Robot, String> {
        Urdf::read(text, "r.urdf")
            .and_then(|m| validate(m, "r.urdf"))
            .map_err(|e| e.message())
    }

    fn fails(text: &str, message: &str) {
        assert_eq!(check(text).err().as_deref(), Some(message));
    }

    #[test]
    fn step_3_the_root_must_be_robot() {
        fails(
            "<model/>",
            "kmr: r.urdf:1: the root element is <model>, expected <robot>",
        );
    }

    #[test]
    fn step_4_floating_is_roadmap() {
        let text = robot(&joint("j1", "floating", "base", "l1", "", ""));
        fails(
            &text,
            "kmr: r.urdf:3: joint j1: floating: roadmap: non-1-DOF joints",
        );
    }

    #[test]
    fn step_5_mimic_is_a_multi_actuator_joint() {
        let mimic = "<mimic joint=\"j1\"/>";
        let text = robot(
            &(joint("j1", "revolute", "base", "l1", LIMIT, "")
                + &joint("j2", "revolute", "l1", "l2", LIMIT, mimic)),
        );
        fails(
            &text,
            "kmr: r.urdf:4: joint j2: <mimic>: SRS index space: multi-actuator joint",
        );
    }

    #[test]
    fn step_5_a_transmission_of_two_joints_is_a_multi_actuator_joint() {
        let t =
            "  <transmission name=\"t1\"><joint name=\"j1\"/><joint name=\"j2\"/></transmission>\n";
        let text = two(LIMIT, LIMIT).replace("</robot>", &format!("{t}</robot>"));
        fails(
            &text,
            "kmr: r.urdf:5: transmission t1: 2 <joint> elements: SRS index space: multi-actuator joint",
        );
    }

    #[test]
    fn step_5_a_transmission_of_one_joint_and_one_actuator_builds() {
        let t = "<transmission name=\"t1\"><joint name=\"j1\"/><actuator name=\"m1\"/></transmission>\n";
        let text = two(LIMIT, LIMIT).replace("</robot>", &format!("{t}</robot>"));
        assert_eq!(check(&text).map(|r| r.actuators.len()), Ok(2));
    }

    #[test]
    fn step_6_duplicate_joint() {
        let text = robot(
            &(joint("j1", "revolute", "base", "l1", LIMIT, "")
                + &joint("j1", "fixed", "l1", "l2", "", "")),
        );
        fails(&text, "kmr: r.urdf:4: duplicate joint j1 (first at line 3)");
    }

    #[test]
    fn step_6_duplicate_link() {
        let text = two(LIMIT, LIMIT).replace("</robot>", "<link name=\"l1\"/>\n</robot>");
        fails(&text, "kmr: r.urdf:5: duplicate link l1 (first at line 2)");
    }

    #[test]
    fn step_7_a_revolute_joint_needs_a_limit() {
        fails(&two(LIMIT, ""), "kmr: r.urdf:4: joint j2: no <limit>");
    }

    #[test]
    fn step_7_a_continuous_joint_needs_effort_only_with_velocity() {
        let text = robot(&joint(
            "j1",
            "continuous",
            "base",
            "l1",
            "<limit velocity=\"1\"/>",
            "",
        ))
        .replace("<link name=\"l2\"/>", "");
        fails(&text, "kmr: r.urdf:3: joint j1: <limit> has no effort");
        let ok = text.replace("velocity=\"1\"", "velocity=\"1\" effort=\"2\"");
        assert_eq!(
            check(&ok).map(|r| (r.actuators[0].lower, r.actuators[0].upper)),
            Ok((0.0, 0.0))
        );
    }

    #[test]
    fn step_7_ranges() {
        let cases = [
            (
                "lower=\"-1\" upper=\"1\"",
                "lower=\"2\" upper=\"1\"",
                "lower > upper",
            ),
            ("velocity=\"2\"", "velocity=\"-2\"", "velocity < 0"),
            ("effort=\"3\"", "effort=\"-3\"", "effort < 0"),
        ];
        for (from, to, reason) in cases {
            let text = two(LIMIT, &LIMIT.replace(from, to));
            fails(&text, &format!("kmr: r.urdf:4: joint j2: {reason}"));
        }
    }

    #[test]
    fn step_8_every_value_fits_f32() {
        let cases = [
            ("upper=\"1\"", "upper=\"inf\"", "upper"),
            ("effort=\"3\"", "effort=\"1e39\"", "effort"),
            ("velocity=\"2\"", "velocity=\"NaN\"", "velocity"),
        ];
        for (from, to, field) in cases {
            let text = two(&LIMIT.replace(from, to), LIMIT);
            fails(
                &text,
                &format!("kmr: r.urdf:3: joint j1: {field} does not fit f32"),
            );
        }
    }

    #[test]
    fn step_8_prismatic_is_converted_to_mm_before_the_f32_check() {
        let limit = "<limit lower=\"0\" upper=\"0.1\" velocity=\"0.05\" effort=\"20\"/>";
        let text = robot(&joint("j1", "prismatic", "base", "l1", limit, ""))
            .replace("<link name=\"l2\"/>", "");
        let a = check(&text).map(|r| r.actuators[0].clone());
        assert_eq!(
            a.map(|a| (a.lower, a.upper, a.speed, a.effort)),
            Ok((0.0, 100.0, 50.0, 20.0))
        );
        let big = text.replace("upper=\"0.1\"", "upper=\"3.5e35\"");
        fails(&big, "kmr: r.urdf:3: joint j1: upper does not fit f32");
    }

    #[test]
    fn step_9_two_roots_are_a_multi_robot_descriptor() {
        let text = robot(&joint("j1", "revolute", "base", "l1", LIMIT, ""));
        fails(
            &text,
            "kmr: r.urdf: 2 root links (base, l2): SRS FR-6.5: multi-robot descriptor",
        );
    }

    #[test]
    fn step_9_a_joint_names_a_known_link() {
        let text = two(LIMIT, LIMIT).replace("<child link=\"l2\"/>", "<child link=\"l9\"/>");
        fails(
            &text,
            "kmr: r.urdf:4: joint j2: child link l9 is not defined",
        );
    }

    #[test]
    fn step_9_a_link_has_one_parent_joint() {
        let text = two(LIMIT, LIMIT).replace(
            "<parent link=\"l1\"/><child link=\"l2\"/>",
            "<parent link=\"base\"/><child link=\"l1\"/>",
        );
        fails(
            &text,
            "kmr: r.urdf:4: joint j2: link l1 is already the child of joint j1",
        );
    }

    #[test]
    fn step_9_a_common_root_for_several_chains_is_one_robot() {
        let text = robot(
            &(joint("j1", "revolute", "base", "l1", LIMIT, "")
                + &joint("j2", "revolute", "base", "l2", LIMIT, "")),
        );
        let parents = check(&text).map(|r| r.links.iter().map(|l| l.parent).collect::<Vec<_>>());
        assert_eq!(parents, Ok(vec![None, Some(0), Some(0)]));
    }

    #[test]
    fn step_10_at_least_one_actuator() {
        let text = robot(
            &(joint("f1", "fixed", "base", "l1", "", "")
                + &joint("f2", "fixed", "l1", "l2", "", "")),
        );
        fails(&text, "kmr: r.urdf: no actuated joint");
    }

    #[test]
    fn two_faults_report_the_earlier_step() {
        let late = two(LIMIT, &LIMIT.replace("effort=\"3\"", "effort=\"-3\""));
        let duplicate = late.replace("name=\"j2\"", "name=\"j1\"");
        fails(
            &duplicate,
            "kmr: r.urdf:4: duplicate joint j1 (first at line 3)",
        );
        let mimic = robot(
            &(joint("j1", "floating", "base", "l1", "", "")
                + &joint("j2", "revolute", "l1", "l2", LIMIT, "<mimic joint=\"j1\"/>")),
        );
        fails(
            &mimic,
            "kmr: r.urdf:3: joint j1: floating: roadmap: non-1-DOF joints",
        );
    }

    #[test]
    fn a_message_without_a_line_drops_it() {
        let e = ModelError::file("a.ron", "no actuated joint");
        assert_eq!(e.message(), "kmr: a.ron: no actuated joint");
        let e = ModelError::element("a.ron", None, "joint j", "lower > upper");
        assert_eq!(e.message(), "kmr: a.ron: joint j: lower > upper");
    }
}
