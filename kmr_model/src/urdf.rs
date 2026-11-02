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
            model.links.push(Link {
                name: read.name(*link, "link")?.to_owned(),
                mass: 0.0,
                com: [0.0; 3],
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
mod tests {
    use super::*;

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
