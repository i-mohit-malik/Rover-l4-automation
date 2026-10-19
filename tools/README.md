# tools/

Laptop-side helpers. Nothing here runs on the ESP32.

| Script | Does |
|---|---|
| `plot_run.py` | Plots one timeline CSV (format: `docs/logging.md`) to a PNG next to it. |
| `fetch_log.py` | Downloads the newest log from the box over Wi-Fi into `L4_Rover/04_SAFETY_AND_TESTING/Test_Results/` with the date-time name (`--out` to change). Needs the `/log` endpoint on the box (planned). |
| `make_submission.ps1` | Builds the CAS source zip with the required file name into `L4_Rover/06_REPORTS_AND_DEMO/Final_Submission/`. |

## Setup

```bash
python -m venv .venv
.venv\Scripts\activate
pip install -r tools/requirements.txt
```

## Usage

```bash
python tools/plot_run.py ../../04_SAFETY_AND_TESTING/Test_Results/2026-10-26_1430_manual-lap1.csv
python tools/fetch_log.py --box 6 --note manual-lap1
```

```powershell
.\tools\make_submission.ps1 -Matriculation 1234567,2345678,3456789
```
