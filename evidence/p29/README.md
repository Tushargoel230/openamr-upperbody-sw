# P29: the AUX race, three A7 runs (28 Oct 2026)

| file | what |
|---|---|
| `clamp_loop_30.txt` | `test_zz_a_reading_beyond_the_epsilon_is_not_hidden` run alone 30 times (the clamp-on launch test, one `colcon test` each), the testcase read from the xunit each time: 30 of 30 pass, none skipped |
| `mutation_check.txt` | the fixed test against two broken clamps (a beyond-epsilon reading also counted as a clamp, every one and only the first), both FAIL; the real clamp passes |
| `reports/20261028T090816Z_a7.json`, `...T091706Z_a7.json`, `...T092556Z_a7.json` | A7 runs 1 to 3 on `c84cfbd`: 14 PASS, 2 WARN (M07, V01), 0 FAIL each; M03 99 disabled / 567 enabled |
| `run_exit_codes.txt` | exit codes, the stop-condition comparison with the clean P28 run, the non-zero process exits |

**Why the reports are here and not in `../reports/`.** `tools/check_evidence_backing.py` derives the run-level crash rate (4 of 24 A7 runs on `dfd44a5`) from every `evidence/reports/*_a7.json`, and `docs/pr/claim_backing.yaml` pins that text. Three more reports there would make the derived rate 4 of 27 and fail the ledger, and the ledger is not something P29 was allowed to edit. When someone may edit it: move these three into `../reports/`, change "4 of 24" to "4 of 27" and "17 percent" to "15 percent" in the PR body and the ledger, and add a P29 section to `../reports/README.md`.

**Root cause and fix of the race**, as in the commit message of `4f2c3b4`: the test compared `activations` read before the value switch with `activations` read after it. The publisher keeps sending the in-epsilon noise until the switch takes effect, each such message is a legitimate clamp, so one in flight when the first number was read makes the two differ by one (42 against 41 in P28 run 2). Nothing beyond the epsilon was counted as a clamp. The test now takes its baseline from the snapshot in which the first beyond-epsilon reading has been counted and checks `max_magnitude_rad` within one snapshot.
