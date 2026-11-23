# What the mdbook will contain

 
## What is kmr3 and how to use it
- Intro
- How to install
- Intro to the "custom ECS" we have
- Complete and extensive API tutorial based on the examples in kmr_api
- How to use lua,py and c/cpp FFIs
- How to use external libs in all FFIs
- How to declare hardware and use it
- Extensive comparaison with ROS2 for the exact same controllers +
proof our architecture is easier to port from robot to robot.
- Extensive benchmarks
- Extensive benchmarks against ROS2 (same controller) and other robotic frameworks

## How to contribute

- What each crate do
- TLDR of the SRS

### As an extensive step by step real implementation. 

This must be a precise and complete guide, a litteral step-by-step of how
we would implment a simple hardware adapter (a real one, not a fake one just
for the sake of it + link to the repo of the simplistic one.)

- How to add a new adapter (for actuators)
- How to add a new adapter (for senors)
 

## Applies to the above

mdBook, with rustdoc only for testing. mdBook turns a SUMMARY.md plus chapter .md files into a static site, parsing Markdown with pulldown-cmark, rendering through Handlebars templates, highlighting code with highlight.js, and making snippets runnable via the Rust Playground. rustdoc comes in only through mdbook test, which compiles and runs the code blocks as doctests. The Book also adds its own preprocessors, like the trpl-listing and Ferris annotations in the rust-lang/book repo. Source: mdBook guide.
