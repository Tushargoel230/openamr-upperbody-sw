# Reports cited by the PR descriptions

Machine-written, copied byte for byte from the author's workspace `evidence/`; they contain local checkout paths. Each describes the commit in its
`git_hash` field, on a clean tree (`dirty: false`). `WARN` is not `PASS`.

| file | what | commit | result |
|---|---|---|---|
| `20261006T164342Z_dev.json` | A6 suite (`verify_a6.sh`), the report PR-1 cites | `993b000` (A6 head) | 20 PASS, 6 WARN, 0 FAIL |
| `20261012T164313Z_dev.json` | A6 suite on the A7 branch (adds H01, H02; V01 now carries the manual-look findings) | `93a2a49` | 22 PASS, 6 WARN, 0 FAIL, 0 NOT_RUN |
| `20261012T170225Z_a7.json` | A7 regression (`run_a7_regression.sh`), the report PR-2 cites | `93a2a49` | 13 PASS, 1 WARN (V01), 0 FAIL, 0 NOT_RUN |
| `20261012T165237Z_a7.json` | the first A7 run on the same commit: **FAILED** M01 (and AUX) through the process guard, `joint_state_clamp.py` exited 1 at shutdown in one of four launches; all poses passed. Kept beside the passing one | `93a2a49` | 11 PASS, 1 WARN, 2 FAIL |

The two A7 reports ran on one commit with nothing changed between them: one of two runs failed, and both are here.

Class: `static+mock` for `_dev`, `simulation` for `_a7`. No physical evidence. The frozen expansion set is in `../frozen_a6_993b000/`.
