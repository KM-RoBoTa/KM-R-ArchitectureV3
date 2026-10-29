---
name: speckit-ov-c4-code
description: Write the C4 component and code levels (C3/C4) for one module in docs/c4/<module>.dsl
compatibility: Requires spec-kit project structure with .specify/ directory
metadata:
  author: TODO(author)
  source: ov:commands/speckit.ov.c4-code.md
---

# ov: C4 components and code for one module

Writes C3 (components) and C4 (code) for **exactly one** SRS module into
`docs/c4/<module>.dsl`. Spec-kit also offers this command as the optional `after_implement`
hook.

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty). If it names a module,
that is the module.

## Execution

1. **Pick the module.** Use the module named in the user input. When none is given (also
   when this runs as the `after_implement` hook), list the modules with
   `.specify/extensions/ov/scripts/bash/ov-srs.sh list` and ask the human which one. When
   running as the hook, suggest the module whose `Paths:` cover the files just implemented.

2. **Make sure the include and the file exist**:

   - **Bash**: `.specify/extensions/ov/scripts/bash/ov-c4.sh includes <module>`

   On exit `1` (unknown module, no SRS, no `docs/workspace.dsl`: run `/speckit-ov-c4-context`
   first), show the message and stop.

3. **Write `docs/c4/<module>.dsl`** from the module's code and its spec
   (`docs/srs/modules/<module>.md`), in Structurizr DSL:
   - add the components to the container declared in `docs/workspace.dsl` with
     `!element <container identifier> { … }` (hierarchical identifiers, e.g.
     `!element system.app { … }`);
   - C3: one `component "<name>" "<description>" "<technology>"` per component, with the
     relationships between them;
   - C4: Structurizr has no code-level element type, so each code element (a main type,
     module or function) is a component tagged `Code`, inside a `group` named after its C3
     component:

     ```text
     !element system.app {
         billing = component "Billing" "Computes invoices" "Rust"
         group "Billing" {
             invoice = component "Invoice" "Invoice type" "Rust" {
                 tags "Code"
             }
         }
     }
     ```

   Replace the stub line. Rewrite the file freely: it is generated for this module.

## Rules

- Touch only `docs/c4/<module>.dsl` for the chosen module. Never edit another module's
  `.dsl`, the SRS, the module specs, or the include region of `docs/workspace.dsl`.