//! URDF descriptors, read with `roxmltree` (research R8).

use roxmltree::{Document, Node};

use crate::model::{
    Actuator, Joint, Kind, Limit, Link, MULTI_ACTUATOR, Model, ModelError, NON_1_DOF, Pos, Reader,
};

/// The URDF reader.
pub(crate) struct Urdf;

impl Reader for Urdf {
    fn read(text: &str, file: &str) -> Result<Model, ModelError> {
        let doc = Document::parse(text).map_err(|e| {
            let at = e.pos();
            let text = e.to_string();
            let suffix = format!(" at {}:{}", at.row, at.col);
            let reason = text.strip_suffix(&suffix).unwrap_or(&text);
            ModelError::syntax(file, (at.row, at.col), reason)
        })?;
        let read = Read { doc: &doc, file };
        let robot = doc.root_element();
        if !robot.has_tag_name("robot") {
            return Err(ModelError {
                line: Some(read.pos(robot).0),
                ..ModelError::file(
                    file,
                    format!(
                        "the root element is <{}>, expected <robot>",
                        robot.tag_name().name()
                    ),
                )
            });
        }
        let links = children(robot, "link");
        let joints = children(robot, "joint");

        let mut model = Model::default();
        for link in &links {
            let name = read.name(*link, "link")?;
            let (mass, com) = read.inertial(*link, name)?;
            model.links.push(Link {
                name: name.to_owned(),
                mass,
                com,
                pos: Some(read.pos(*link)),
            });
        }
        let mut named = Vec::with_capacity(joints.len());
        for joint in &joints {
            named.push((*joint, read.name(*joint, "joint")?));
        }
        let mut kinds = Vec::with_capacity(joints.len());
        for (joint, name) in &named {
            kinds.push(read.kind(*joint, name)?);
        }
        for (joint, name) in &named {
            if let Some(mimic) = children(*joint, "mimic").first() {
                return Err(read
                    .fault(*mimic, "joint", name, "<mimic>")
                    .marked(MULTI_ACTUATOR));
            }
        }
        for t in children(robot, "transmission") {
            read.transmission(t)?;
        }
        for ((joint, name), kind) in named.iter().zip(kinds) {
            model.joints.push(Joint {
                name: (*name).to_owned(),
                parent: read.end(*joint, name, "parent")?.to_owned(),
                child: read.end(*joint, name, "child")?.to_owned(),
                pos: Some(read.pos(*joint)),
            });
            if let Some(kind) = kind {
                model.actuators.push(Actuator {
                    name: (*name).to_owned(),
                    kind,
                    limit: read.limit(*joint, name)?,
                    pos: Some(read.pos(*joint)),
                });
            }
        }
        Ok(model)
    }
}

/// The element children of `node` named `tag`, in document order.
fn children<'a, 'i>(node: Node<'a, 'i>, tag: &str) -> Vec<Node<'a, 'i>> {
    node.children().filter(|c| c.has_tag_name(tag)).collect()
}

/// One URDF document being read, for positions and errors.
struct Read<'a, 'i> {
    /// The parsed document.
    doc: &'a Document<'i>,
    /// The path shown in errors.
    file: &'a str,
}

impl<'a, 'i> Read<'a, 'i> {
    /// Where `node` starts.
    fn pos(&self, node: Node) -> Pos {
        let at = self.doc.text_pos_at(node.range().start);
        (at.row, at.col)
    }

    /// A fault of the element `what name` at `node`.
    fn fault(&self, node: Node, what: &str, name: &str, reason: &str) -> ModelError {
        ModelError::element(
            self.file,
            Some(self.pos(node)),
            format!("{what} {name}"),
            reason,
        )
    }

    /// The `name` attribute of a `<link>` or `<joint>`.
    fn name(&self, node: Node<'a, 'i>, what: &str) -> Result<&'a str, ModelError> {
        node.attribute("name").ok_or_else(|| {
            ModelError::element(
                self.file,
                Some(self.pos(node)),
                format!("<{what}>"),
                "no name",
            )
        })
    }

    /// The actuated kind of a joint, `None` when fixed.
    fn kind(&self, joint: Node, name: &str) -> Result<Option<Kind>, ModelError> {
        match joint.attribute("type") {
            Some("fixed") => Ok(None),
            Some("revolute") => Ok(Some(Kind::Revolute)),
            Some("continuous") => Ok(Some(Kind::Continuous)),
            Some("prismatic") => Ok(Some(Kind::Prismatic)),
            Some(t @ ("floating" | "planar")) => {
                Err(self.fault(joint, "joint", name, t).marked(NON_1_DOF))
            }
            Some(other) => Err(self.fault(joint, "joint", name, &format!("unknown type {other}"))),
            None => Err(self.fault(joint, "joint", name, "no type")),
        }
    }

    /// Refuses a transmission that couples several joints or actuators.
    fn transmission(&self, t: Node) -> Result<(), ModelError> {
        let name = t.attribute("name").unwrap_or("(unnamed)");
        for tag in ["joint", "actuator"] {
            let count = children(t, tag).len();
            if count > 1 {
                return Err(self
                    .fault(
                        t,
                        "transmission",
                        name,
                        &format!("{count} <{tag}> elements"),
                    )
                    .marked(MULTI_ACTUATOR));
            }
        }
        Ok(())
    }

    /// The link named by a joint's `<parent>` or `<child>`.
    fn end(&self, joint: Node<'a, 'i>, name: &str, tag: &str) -> Result<&'a str, ModelError> {
        children(joint, tag)
            .first()
            .and_then(|n| n.attribute("link"))
            .ok_or_else(|| self.fault(joint, "joint", name, &format!("no <{tag} link>")))
    }

    /// A link's mass (kg) and mass center (m, link frame); without
    /// `<inertial>`, mass 0 at the link origin, the URDF default.
    fn inertial(&self, link: Node, name: &str) -> Result<(f64, [f64; 3]), ModelError> {
        let Some(inertial) = children(link, "inertial").first().copied() else {
            return Ok((0.0, [0.0; 3]));
        };
        let fault = |node: Node, reason: &str| self.fault(node, "link", name, reason);
        let mass = children(inertial, "mass")
            .first()
            .and_then(|m| m.attribute("value").map(|v| (*m, v)))
            .ok_or_else(|| fault(inertial, "<inertial> has no <mass value>"))?;
        let mass = mass.1.trim().parse::<f64>().map_err(|_| {
            fault(
                mass.0,
                &format!("<mass value=\"{}\"> is not a number", mass.1),
            )
        })?;
        let Some((origin, xyz)) = children(inertial, "origin")
            .first()
            .and_then(|o| o.attribute("xyz").map(|v| (*o, v)))
        else {
            return Ok((mass, [0.0; 3]));
        };
        let com = xyz
            .split_whitespace()
            .map(str::parse)
            .collect::<Result<Vec<f64>, _>>()
            .ok()
            .and_then(|v| <[f64; 3]>::try_from(v).ok())
            .ok_or_else(|| {
                fault(
                    origin,
                    &format!("<origin xyz=\"{xyz}\"> is not three numbers"),
                )
            })?;
        Ok((mass, com))
    }

    /// A joint's `<limit>`, each attribute that is there read as a number.
    fn limit(&self, joint: Node, name: &str) -> Result<Option<Limit>, ModelError> {
        let Some(limit) = children(joint, "limit").first().copied() else {
            return Ok(None);
        };
        let number = |attr: &str| -> Result<Option<f64>, ModelError> {
            limit
                .attribute(attr)
                .map(|v| {
                    v.trim().parse::<f64>().map_err(|_| {
                        self.fault(
                            limit,
                            "joint",
                            name,
                            &format!("<limit {attr}=\"{v}\"> is not a number"),
                        )
                    })
                })
                .transpose()
        };
        Ok(Some(Limit {
            lower: number("lower")?,
            upper: number("upper")?,
            velocity: number("velocity")?,
            effort: number("effort")?,
        }))
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// kmr_api/tests/fixtures/four.urdf, the reference robot, inline.
    pub(crate) const FOUR: &str = r#"<?xml version="1.0"?>
<robot name="four">
  <link name="base"><inertial><mass value="0.5"/><origin xyz="0 0 0.02"/></inertial></link>
  <link name="l1"><inertial><mass value="0.35"/><origin xyz="0 0 0.05"/></inertial></link>
  <link name="l2"><inertial><mass value="0.3"/><origin xyz="0 0 0.04"/></inertial></link>
  <link name="l3"><inertial><mass value="0.2"/><origin xyz="0 0 0.03"/></inertial></link>
  <link name="l4"><inertial><mass value="0.1"/><origin xyz="0 0 0.02"/></inertial></link>
  <link name="tool"><inertial><mass value="0.05"/><origin xyz="0 0 0.01"/></inertial></link>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="l1"/>
    <limit lower="-1.57" upper="1.57" velocity="3.0" effort="1.5"/>
  </joint>
  <joint name="j2" type="continuous">
    <parent link="l1"/><child link="l2"/>
    <limit velocity="3.0" effort="1.5"/>
  </joint>
  <joint name="j3" type="prismatic">
    <parent link="l2"/><child link="l3"/>
    <limit lower="0.0" upper="0.1" velocity="0.05" effort="20"/>
  </joint>
  <joint name="j4" type="revolute">
    <parent link="l3"/><child link="l4"/>
    <limit lower="-3.14" upper="3.14" velocity="4.0" effort="2.0"/>
  </joint>
  <joint name="mount" type="fixed"><parent link="l4"/><child link="tool"/></joint>
  <transmission name="t1"><joint name="j1"/><actuator name="m1"/></transmission>
</robot>
"#;

    /// The checked robot of [`FOUR`].
    pub(crate) fn four() -> crate::model::Robot {
        let robot = Urdf::read(FOUR, "four.urdf")
            .and_then(|m| crate::model::validate(m, "four.urdf"))
            .map_err(|e| e.message());
        match robot {
            Ok(robot) => robot,
            Err(e) => panic!("{e}"),
        }
    }

    /// The link named `name` of the checked [`FOUR`].
    fn link(name: &str) -> crate::model::CheckedLink {
        let robot = four();
        let found = robot.links.iter().find(|l| l.name == name).cloned();
        match found {
            Some(link) => link,
            None => panic!("no link {name}"),
        }
    }

    #[test]
    fn a_link_has_its_mass_and_mass_center() {
        let l1 = link("l1");
        assert_eq!((l1.mass, l1.com), (0.35, [0.0, 0.0, 0.05]));
    }

    #[test]
    fn a_link_hangs_from_its_joint_parent_link() {
        let robot = four();
        let parents: Vec<_> = robot.links.iter().map(|l| l.parent).collect();
        assert_eq!(parents, [None, Some(0), Some(1), Some(2), Some(3), Some(4)]);
    }

    #[test]
    fn a_link_without_inertial_has_no_mass_at_its_origin() {
        let model = read(&chain(&[("j", "revolute")]));
        let l0 = model.map(|m| (m.links[0].mass, m.links[0].com));
        assert_eq!(l0, Ok((0.0, [0.0; 3])));
    }

    #[test]
    fn an_inertial_without_origin_puts_the_mass_center_at_the_link_origin() {
        let text = FOUR.replace("<origin xyz=\"0 0 0.05\"/>", "");
        let model = read(&text);
        assert_eq!(
            model.map(|m| (m.links[1].mass, m.links[1].com)),
            Ok((0.35, [0.0; 3]))
        );
    }

    #[test]
    fn a_bad_inertial_names_the_link() {
        let cases = [
            (
                "<mass value=\"0.35\"/>",
                "",
                "<inertial> has no <mass value>",
            ),
            (
                "value=\"0.35\"",
                "value=\"heavy\"",
                "<mass value=\"heavy\"> is not a number",
            ),
            (
                "xyz=\"0 0 0.05\"",
                "xyz=\"0 0\"",
                "<origin xyz=\"0 0\"> is not three numbers",
            ),
        ];
        for (from, to, reason) in cases {
            let err = read(&FOUR.replace(from, to));
            assert_eq!(err, Err(format!("kmr: r.urdf:4: link l1: {reason}")));
        }
    }

    #[test]
    fn a_mass_that_is_not_finite_fails_step_8() {
        let text = FOUR.replace("value=\"0.35\"", "value=\"inf\"");
        let err = Urdf::read(&text, "r.urdf")
            .and_then(|m| crate::model::validate(m, "r.urdf"))
            .map_err(|e| e.message());
        assert_eq!(
            err,
            Err("kmr: r.urdf:4: link l1: mass does not fit f32".to_owned())
        );
    }

    /// A `<robot>` around `body`.
    fn robot(body: &str) -> String {
        format!("<?xml version=\"1.0\"?>\n<robot name=\"t\">\n{body}\n</robot>\n")
    }

    /// A joint of `kind` from `parent` to `child`, with a full limit.
    fn joint(name: &str, kind: &str, parent: &str, child: &str) -> String {
        format!(
            "<joint name=\"{name}\" type=\"{kind}\"><parent link=\"{parent}\"/><child link=\"{child}\"/>\
             <limit lower=\"-1\" upper=\"1\" velocity=\"2\" effort=\"3\"/></joint>"
        )
    }

    /// Links `l0` to `l<n>` and one joint per `kinds` entry, `l<i>` to `l<i+1>`.
    fn chain(kinds: &[(&str, &str)]) -> String {
        let mut body: String = (0..=kinds.len())
            .map(|i| format!("<link name=\"l{i}\"/>"))
            .collect();
        for (i, (name, kind)) in kinds.iter().enumerate() {
            body += &joint(name, kind, &format!("l{i}"), &format!("l{}", i + 1));
        }
        robot(&body)
    }

    fn read(text: &str) -> Result<Model, String> {
        Urdf::read(text, "r.urdf").map_err(|e| e.message())
    }

    fn names(model: &Model) -> Vec<&str> {
        model.actuators.iter().map(|a| a.name.as_str()).collect()
    }

    #[test]
    fn four_revolute_joints_are_four_actuators_in_file_order() {
        let kinds = [
            ("d", "revolute"),
            ("b", "revolute"),
            ("c", "revolute"),
            ("a", "revolute"),
        ];
        let model = read(&chain(&kinds));
        assert_eq!(model.as_ref().map(names), Ok(vec!["d", "b", "c", "a"]));
    }

    #[test]
    fn fixed_joints_get_no_index() {
        let kinds = [
            ("a1", "revolute"),
            ("f1", "fixed"),
            ("a2", "revolute"),
            ("a3", "prismatic"),
            ("a4", "continuous"),
            ("f2", "fixed"),
            ("a5", "revolute"),
            ("a6", "revolute"),
        ];
        let model = read(&chain(&kinds));
        assert_eq!(
            model.as_ref().map(names),
            Ok(vec!["a1", "a2", "a3", "a4", "a5", "a6"])
        );
        assert_eq!(model.map(|m| m.joints.len()), Ok(8));
    }

    #[test]
    fn one_continuous_joint_is_one_actuator() {
        let model = read(&chain(&[("spin", "continuous")]));
        assert_eq!(
            model.map(|m| m.actuators.iter().map(|a| a.kind).collect::<Vec<_>>()),
            Ok(vec![Kind::Continuous])
        );
    }

    #[test]
    fn limits_and_positions_are_read() {
        let model = read(&chain(&[("j", "revolute")]));
        let a = model.as_ref().map(|m| &m.actuators[0]);
        let limit = Limit {
            lower: Some(-1.0),
            upper: Some(1.0),
            velocity: Some(2.0),
            effort: Some(3.0),
        };
        assert_eq!(
            a.map(|a| (a.limit, a.pos)),
            Ok((Some(limit), Some((3, 35))))
        );
    }

    #[test]
    fn a_joint_without_a_name_names_the_file() {
        let text = robot("<link name=\"a\"/><joint type=\"fixed\"/>");
        assert_eq!(
            read(&text),
            Err("kmr: r.urdf:3: <joint>: no name".to_owned())
        );
    }

    #[test]
    fn a_link_without_a_name_names_the_file() {
        let text = robot("<link/>");
        assert_eq!(
            read(&text),
            Err("kmr: r.urdf:3: <link>: no name".to_owned())
        );
    }

    #[test]
    fn floating_and_planar_are_roadmap() {
        for kind in ["floating", "planar"] {
            let err = read(&chain(&[("j", kind)]));
            assert_eq!(
                err,
                Err(format!(
                    "kmr: r.urdf:3: joint j: {kind}: roadmap: non-1-DOF joints"
                ))
            );
        }
    }

    #[test]
    fn an_unknown_or_missing_type_names_the_joint() {
        assert!(
            read(&chain(&[("j", "ball")]))
                .is_err_and(|e| e.ends_with("joint j: unknown type ball"))
        );
        let text = robot("<link name=\"a\"/><joint name=\"j\"/>");
        assert!(read(&text).is_err_and(|e| e.ends_with("joint j: no type")));
    }

    #[test]
    fn a_limit_that_is_not_a_number_names_joint_and_attribute() {
        let text = chain(&[("j", "revolute")]).replace("effort=\"3\"", "effort=\"lots\"");
        assert!(
            read(&text)
                .is_err_and(|e| e.contains("joint j: <limit effort=\"lots\"> is not a number"))
        );
    }

    #[test]
    fn the_root_must_be_robot() {
        let err = read("<model name=\"t\"/>");
        assert_eq!(
            err,
            Err("kmr: r.urdf:1: the root element is <model>, expected <robot>".to_owned())
        );
    }

    #[test]
    fn a_syntax_error_has_line_and_column() {
        let err = read("<robot>\n<link name=\"a\">\n</robot>");
        assert_eq!(
            err,
            Err("kmr: r.urdf:3:1: expected 'link' tag, not 'robot'".to_owned())
        );
    }
}
