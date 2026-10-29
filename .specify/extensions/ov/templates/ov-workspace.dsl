/*
 * docs/workspace.dsl: the C4 model of this system, in Structurizr DSL.
 *
 * This file is shared: the human and the agent may both edit it.
 * - C1 (system context) and C2 (containers) live here, built with /speckit-ov-c4-context.
 * - C3 (components) and C4 (code) live in docs/c4/<module>.dsl, one file per SRS module,
 *   written by /speckit-ov-c4-code and included below.
 * Keep the two ov:includes markers: ov-c4.sh maintains the !include lines between them.
 * Replace every TODO placeholder with your own text.
 */
workspace "TODO(system name)" "TODO(one-line description)" {

    !identifiers hierarchical

    model {
        // C1: people and software systems
        user = person "TODO(user)" "TODO(who they are)"
        system = softwareSystem "TODO(system name)" "TODO(what it does)" {
            // C2: containers (applications and data stores)
            app = container "TODO(container)" "TODO(what it does)" "TODO(technology)"
        }
        user -> system.app "TODO(uses it to)"

        // C3/C4 per module (docs/c4/<module>.dsl)
        // ov:includes:begin
        // ov:includes:end
    }

    views {
        systemContext system "SystemContext" {
            include *
            autoLayout lr
        }

        container system "Containers" {
            include *
            autoLayout lr
        }

        // No `theme default`: it loads from static.structurizr.com, which is gone since the
        // Structurizr cloud service's end of life (2026-09-30), so validation would fail.
    }
}
