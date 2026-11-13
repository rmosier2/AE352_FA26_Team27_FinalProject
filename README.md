# Drone Propulsion Sizing (AE 352 Team Project, Fall 2026)

Steady-state propulsion and battery sizing tools for the quadcopter digital-twin
project. Given a vehicle mass and a mission profile (a list of thrust-to-weight
ratios held for set durations), the model computes the propeller RPM, motor and
ESC operating point, and battery charge/energy drawn, and checks whether the
mission fits within the usable battery capacity (ER3: achieve the performance
goals without exhausting the power supply).

## Repository contents

| File | Purpose |
|---|---|
| `FlightTimeModel.py` | Core model: the `Drone` class (propeller, motor, ESC, and battery chain). |
| `FlightTimeModel_runcase.ipynb` | Runs the mission for one fixed vehicle (MTOW 1.839 kg and TOW 1.50 kg) and prints a per-segment table. |
| `WeightSweep.ipynb` | Sweeps vehicle weight, finds the maximum weight that completes the mission on the usable battery capacity, and plots mission charge vs. weight. |
| `requirements.txt` | Python dependencies. |

## Setup

Requires **Python 3.12 or newer** (`WeightSweep.ipynb` uses f-strings that reuse
the same quote character inside the braces, which older versions reject).

```bash
python -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate
pip install -r requirements.txt
jupyter notebook
```

Open either notebook and run all cells. Both import `FlightTimeModel`, so launch
Jupyter from the repository root.

`WeightSweep.ipynb` saves `weight_sweep.png` in the working directory.

## Baseline configuration

| Item | Value |
|---|---|
| Motor | Hacker A20-50 S Evo, KV 1088 rpm/V, I0 = 0.4 A, R = 0.232 ohm |
| ESC resistance | 0.015 ohm |
| Battery | 3S, 3.7 V/cell (constant), 2.2 Ah, 0.005 ohm/cell |
| Usable capacity | 80% of rated (20% reserve) |
| Propeller | APC 9x6 SF, static (J = 0) CT and CP tables vs. RPM |
| Air density | 1.225 kg/m^3 (ISA sea level) |
| Motors | 4 |

## How the model works

For a requested thrust-to-weight ratio (TWR), each operating point is solved as follows:

1. **Thrust to RPM.** Thrust per motor is `TWR * m * g / n_motors`. Since `T = CT(RPM) * rho * n^2 * D^4` and CT depends on RPM, RPM is found with a root finder (`brentq`). CT and CP are linearly interpolated from the APC static-thrust table.
2. **Shaft power and torque.** `P = CP * rho * n^3 * D^5`, `Q = P / (2 pi n)`.
3. **Motor electrical state.** `I_motor = Q / Kq + I0` with `Kq = 60 / (2 pi KV)`, and `V_motor = I_motor * R_motor + RPM / KV`.
4. **ESC duty cycle.** Solves `V_motor = d * V_pack - d^2 * I_motor * R_eff`, where `R_eff = n_motors * R_pack + R_esc`, for the throttle `d`. The point is rejected (`ValueError`) if the battery cannot supply it (voltage sag) or `d > 1`.
5. **Battery side.** Battery current is `n_motors * d * I_motor`; bus voltage is `V_pack - I_batt * R_pack`; power is `V_bus * I_batt`.

Segments are integrated at constant current for their duration to give charge (Ah)
and energy (Wh). A flag is printed when the required RPM falls outside the
propeller data table (values then clamp to the table ends).

### Key API

```python
from FlightTimeModel import Drone

drone = Drone(weight_kg=1.50)            # total vehicle mass (TOW) OR 1.839 kg for MTOW
drone.operating_point(twr=1.0)            # dict of RPM, throttle, currents, power
drone.segment(twr=1.0, duration_s=60)     # operating point + charge/energy used
drone.run_profile(mission)                # mission = [(label, twr, seconds), ...]
drone.endurance_s(1.0)                    # prints hover endurance on usable capacity
```

## Mission profile used

Derived from the project performance goals (hover 2 min, 1 min circle, 5 m legs
with a 90 degree yaw, slow landing):

| Segment | TWR | Duration (s) |
|---|---|---|
| takeoff/climb | 1.10 | 3 |
| hover 2 min | 1.00 | 120 |
| circle 1 min | 1.00 | 60 |
| accel leg 1 | 1.02 | 3 |
| cruise leg 1 | 1.00 | 5 |
| hover/yaw | 1.00 | 6 |
| leg 2 | 1.00 | 10 |
| main descent | 0.90 | 5 |
| descent 1cm/s | 0.98 | 10 |

## Results (as saved in the notebooks)

- At MTOW **1.839 kg** the 222 s mission uses **1.759 Ah**, about 80% of the 2.2 Ah pack, which is right at the usable limit (margin 0.001 Ah).
- Chosen TOW is **1.50 kg**, which uses **1.238 Ah**, about 56% of the 2.2 Ah pack.
- Hover current is about 20.13 A at about 5091 RPM and 61.4% throttle; hover endurance on usable capacity is about 5.2 min.
- Peak per-arm currents at that weight: 9.04 A (ESC to motor), 5.93 A (battery to ESC).
- In the sweep, weights of about 2.6 kg and above are not flyable (throttle would exceed 100%).

## Limitations and notes

- Segments are **quasi-steady** (constant TWR). The model sizes propulsion and energy; it does not simulate the 6-DOF rigid-body dynamics, drag, or forward-flight propeller effects (static J = 0 data only).
- Battery cell voltage is constant, so there is no discharge-curve model.
- Control-lag and motor transients are ignored.
- Prop data tables cover roughly 2397 to 6540 RPM.
