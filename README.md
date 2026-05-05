# AUV Depth & Course Control using MSS MATLAB Toolbox

**Chris Hunt & Jack Kamataris**

PID depth and course controllers for the REMUS 100 AUV, built in MATLAB/Simulink using the [MSS toolbox](https://github.com/cybergalactic/MSS). Includes a custom stochastic ocean current disturbance model and a sideslip-compensating course autopilot that keeps the vehicle on its intended ground track under realistic cross-current conditions.

---

## What's in here

```
AUVdepthHeadingControlWithRealisticWind.slx   main Simulink model
RunTests2.m                                   batch test runner (8 scenarios x 3 seeds)
GeneratePlots.m                               generates and saves all figures
figures/                                      output folder for saved plots (auto-created)
```

---

## Dependencies

- MATLAB R2024b or later
- Simulink
- [MSS Toolbox](https://github.com/cybergalactic/MSS) clone and add to your MATLAB path

---

## How to run

**1. Install MSS and add it to your path**
```matlab
addpath(genpath('path/to/MSS'))
```

**2. Run the test suite**

Open `RunTests2.m` and set `tests_to_run` to whichever tests you want, then run it. Each test runs across 3 random seeds and saves a `.mat` file per seed to the working directory.

```matlab
% run all 8 tests
tests_to_run = 1:8;

% or just a few
tests_to_run = [3, 5, 7];
```

**3. Generate plots**

Once the `.mat` files exist, run `GeneratePlots.m` from the same directory. Figures are saved as `.png` to a `/figures/` subfolder.

```matlab
GeneratePlots
```

---

## Test scenarios

| # | Name | Wind | Depth | Course |
|---|------|------|-------|--------|
| 1 | Wind Off | Off | 30m hold | 0° |
| 2 | High Disturbance Maintaining Heading | Stormy | 5m hold | heading only |
| 3 | High Disturbance Maintaining Course | Stormy | 5m hold | 0° |
| 4 | High Disturbance Maneuvering | Stormy | 5m hold | -60° |
| 5 | The Ekman Corkscrew | Stormy | 1m → 50m | 0° |
| 6 | The Surface Breach | Stormy | 100m → 5m | 0° |
| 7 | Combined Ascent and Maneuver | Stormy | 100m → 5m | -60° |
| 8 | The Silent Windup | Calm | 30m hold | 0° |

Tests 2 and 3 are intentionally paired where test 2 uses heading-only control, test 3 uses the course autopilot. Comparing their XY plots shows the effect of sideslip compensation.

---

## Adding your own simulation

To add a new test, append a struct to the `tests` array in `RunTests2.m`:

```matlab
tests(9) = struct('name', 'My Test', 'wind', Wind.Normal, 'ctrl', ControlMode.Course, ...
    'z0', 10, 'zf', 10, 'h0', 0, 'hf', 0, 'Kpz', kp, 'Kiz', ki, 'Kdz', kd);
tests_to_run = 9;
```

To swap in a different vehicle plant, replace the REMUS 100 block in the Simulink model with your own S-function. The current disturbance input port and signal logging names (`z`, `x`, `y`, `yaw`, `Pre_Wrap_Current_Direction`) will need to match what `GeneratePlots.m` expects.

---

## Contact

Jack Kamataris — jakamataris@wpi.edu  
Chris Hunt — cdhunt@wpi.edu