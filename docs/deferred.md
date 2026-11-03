# Deferred

_Written by agents when a task hits a forbidden decision (see
`CLAUDE.md`), and by me at step 9. Each entry re-enters the cycle at
step 4. Three open entries on one module = `needs contract rework`.
Never delete a row; set status `done` with the date._

| date | module | step | what was needed | why forbidden | smallest unblocking change | status |
|---|---|---|---|---|---|---|
| 2026-11-03 | bench | T041 | compile-time.sh run natively on the reference Pi, report committed | needs hardware/human: the reference RP4 (SRS §8) | a human runs `bench/scripts/compile-time.sh` on the Pi and commits the report | open |
| 2026-11-03 | bench | T044 | first reference set with all.sh on both kernels, by a non-author, timed from a flashed board | needs hardware/human: the reference RP4 and a person who did not write the bench (SC-001, SC-004) | that person follows bench/PROCEDURE.md "Checklist" and commits bench/reports/ | open |
| 2026-11-03 | bench | T045 | repeatability: cell.sh idle and budget.sh three times each on one kernel | needs hardware/human: the reference RP4 (SC-002, SC-003) | a human runs them and `results compare`/`summary` on the reports | open |
| 2026-11-03 | bench | T060 | hardware runs on the XmBench chain (cell.sh, budget.sh --hw --write all on tick_hw), Torque_Enable check, reports committed | needs hardware/human: the reference chain; also waits on T057–T059 (core K3.5, K3.8) | a human runs them once T057–T059 land | open |
| 2026-11-03 | bench | T063 | presort.sh --gain on the reference chain, optionally the two-bus chain | needs hardware/human: the reference chain; also waits on T061 (core K6.3) | a human runs it once T061 lands; SERIAL_2 in tick_hw.rs set for two-bus | open |
| 2026-11-03 | bench | T066 | runs of the zenoh and ffi bench builds on the reference bench, handed to kmr_api Z4 and kmr_ffi E3 | needs hardware/human: the reference RP4; also waits on T064, T065 | a human runs them once T064, T065 land | open |
| 2026-11-03 | bench | T067 | acceptance of docs/decisions/0006-bench-method.md (status proposed) | needs hardware/human: only the maintainer accepts an ADR | the maintainer sets its status to accepted | open |
