# P24 evidence

| file | what |
|---|---|
| `pytest_process_exits.txt` | the tests of the recording, including the canary: a synthetic -11 planted in a sidecar comes out of the A7 report as a rate that `check_evidence_backing.py` derives and backs; other numbers are a MISMATCH |
| `evidence_backing_canary.txt` | the planted faults of the backing check, including the synthetic `launch_exits` -11 case |
| `evidence_backing_run.txt` | the PR body and the newest reports; `0 of 12` launches with a `parameter_bridge` ending in -11 is BACKED from `process_exits`, the three earlier occurrences are OBSERVED-NOT-RECORDED |
| `decision_currency_run.txt` | the currency check: 0 STALE |
| `pytest_tools.txt` | the tests of the two tools |

The A7 reports themselves are `../reports/20261020T120539Z_a7.json` and the two after it; their `process_exits` is the data.
