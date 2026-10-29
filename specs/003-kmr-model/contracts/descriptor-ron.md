# Contract: `.ron` and JSON descriptor schema

One schema, two syntaxes (spec default). Model data only: no group, no hardware, no port
(FR-005, FR-006; SRS FR-6.1, FR-6.2). Unknown fields are errors (`deny_unknown_fields`).
Units are the URDF ones (rad, m, kg, N·m, N); the model converts prismatic to mm (FR-017).

```ron
Robot(
    name: "xm_bench",
    links: [
        (name: "base",  mass: 0.50, com: (0.0, 0.0, 0.02)),
        (name: "link1", mass: 0.35, com: (0.0, 0.0, 0.05)),
    ],
    joints: [
        (
            name: "shoulder",
            kind: Revolute,          // Revolute | Continuous | Prismatic | Fixed
            parent: "base",
            child: "link1",
            limit: Some((lower: -1.57, upper: 1.57, velocity: 3.0, effort: 1.5)),
        ),
    ],
)
```

The same robot in JSON:

```json
{
  "name": "xm_bench",
  "links": [
    { "name": "base", "mass": 0.50, "com": [0.0, 0.0, 0.02] },
    { "name": "link1", "mass": 0.35, "com": [0.0, 0.0, 0.05] }
  ],
  "joints": [
    {
      "name": "shoulder", "kind": "Revolute", "parent": "base", "child": "link1",
      "limit": { "lower": -1.57, "upper": 1.57, "velocity": 3.0, "effort": 1.5 }
    }
  ]
}
```

Rules:

- Actuator order is the order of `joints`, `Fixed` skipped (FR-003).
- `limit` is required for every non-fixed joint. `lower` and `upper` are optional fields:
  required for `Revolute` and `Prismatic`, rejected for `Continuous`.
- No `origin`, `axis` or geometry yet: the visualizer is roadmap (SRS FR-9.1). Adding them later
  is a MINOR schema change.
- Composing several URDFs in one `.ron` is SRS FR-6.5, open: not in this schema.
