import math
from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import brentq

G = 9.80665 # gravitational constant at sea level (m/s^2)

# ---- APC 9x6 SF Static Thrust Data at Advance Ratio J = 0.0 ----
PROP_RPM = np.array([2397, 2633, 2972, 3220, 3529, 3771, 4050, 4323,
                     4599, 4878, 5137, 5416, 5696, 5975, 6235,  6540], dtype=float)
PROP_CT = np.array([0.1396, 0.1407, 0.105, 0.1424, 0.1444, 0.1470, 0.1484, 0.1489,
                    0.1500, 0.1517, 0.1529, 0.1537, 0.1557, 0.1565, 0.1577, 0.1597], dtype=float)
PROP_CP = np.array([0.0687, 0.0688, 0.0688, 0.0698, 0.0712, 0.0730, 0.0739, 0.0746,
                    0.0756, 0.0772, 0.0783, 0.0794, 0.0809, 0.0821, 0.0834, 0.0851], dtype=float)

def ct_of_rpm(rpm):
    return float(np.interp(rpm, PROP_RPM, PROP_CT))
def cp_of_rpm(rpm):
    return float(np.interp(rpm, PROP_RPM, PROP_CP))

@dataclass
class Drone:
    weight_kg: float       # TOTAL vehicle mass (MTOW) (kg)
    n_motors: int = 4
    
    # Motor / ESC: Hacker A20-50 S Evo KV1088
    kv: float = 1088.0        # rpm/V
    i0: float = 0.4           # zero-load current (A)
    r_motor: float = 0.232    # internal resistance of one motor (ohm)
    r_esc: float = 0.015      # internal resistance of one ESC (ohm)

    # Battery
    cell_voltage: float = 3.7 # voltage of one cell, constant (V)
    n_cells: int = 3 # number of cells in pack
    r_cell: float = 0.005 # internal resistance of one cell (ohm)
    capacity_ah: float = 2.2 # total amp-hours in pack (Ah)
    usable_fraction: float = 0.8 # reserve 20% of battery for optimal usage

    # Propeller and Air
    prop_diameter_m: float = 9 * 0.0254 # 9 inch prop
    rho: float = 1.225 # ISA SLS air density (kg/m^3)

    # Derived quantities
    kq: float = field(init=False)
    v_pack: float = field(init=False)
    r_pack: float = field(init=False)

    def __post_init__(self):
        self.kq = 60.0 / (2 * math.pi * self.kv)    # N*m/A
        self.v_pack = self.cell_voltage * self.n_cells
        self.r_pack = self.r_cell * self.n_cells

    # ---------- Single Operating Point ----------
    def operating_point(self, twr:float) -> dict: 
        """Steady-state values for a given TWR. Raises ValueError if unachievable."""
        D = self.prop_diameter_m
        thrust_per_motor = twr * self.weight_kg * G / self.n_motors # N

        # Step 1: Calculate RPM that produces required thrust
        # Since CT depends on RPM, we use root find
        def thrust_err(rpm):
            n = rpm / 60.0
            return ct_of_rpm(rpm) * self.rho * n**2 * D**4 - thrust_per_motor
        
        rpm = brentq(thrust_err, 1.0, 30000.0)
        out_of_table = not (PROP_RPM[0] <= rpm <= PROP_RPM[-1])

        n = rpm / 60.0
        p_shaft = cp_of_rpm(rpm) * self.rho * n**3 * D**5    # W per motor
        torque = p_shaft / (2 * math.pi * n)                 # N*m per motor

        # Step 2: find motor current and voltage
        i_motor = torque / self.kq + self.i0                 # A
        v_motor = i_motor * self.r_motor + rpm / self.kv     # V

        # Step 3: ESC duty cycle: Vm = d*Vpack - d^2 * I * R_eff
        r_eff = self.n_motors * self.r_pack + self.r_esc
        a,b,c = i_motor * r_eff, -self.v_pack, v_motor
        disc = b**2 - 4*a*c
        if disc < 0:
            raise ValueError(f'TWR {twr:.2f}: battery cannot supply this operating point '
                             f'(voltage sag too large).')
        duty = (-b - math.sqrt(disc)) / (2*a)
        if duty > 1.0:
            raise ValueError(f'TWR {twr:.2f}: needs throttle {duty*100:.0f}% (>100%).')

        i_batt_motor = duty * i_motor                        # per motor, battery side
        i_batt_total = self.n_motors * i_batt_motor
        v_bus = self.v_pack - i_batt_total * self.r_pack
        p_batt = v_bus * i_batt_total

        return dict(twr=twr, throttle=duty, rpm=rpm,
                    thrust_per_motor_N = thrust_per_motor, 
                    shaft_power_per_motor_W = p_shaft, torque_NM = torque,
                    motor_current_A = i_motor, esc_input_current_A = i_batt_motor,
                    battery_current_A = i_batt_total,
                    battery_power_W = p_batt, out_of_prop_table = out_of_table)

    # --------------- one segment ---------------
    def segment(self, twr: float, duration_s: float) -> dict:
        """Operating point for `duration_s` seconds at constant TWR, plus charge/energy used."""
        op = self.operating_point(twr)
        op["duration_s"] = duration_s
        op["charge_Ah"] = op["battery_current_A"] * duration_s / 3600.0
        op["energy_Wh"] = op["battery_power_W"] * duration_s / 3600.0
        return op

    # -------- piecewise mission definition --------
    def run_profile(self, profile, verbose=True) -> dict:
        """profile = [(twr, seconds), ...]  or  [(label, twr, seconds), ...]"""
        segs, total_s, total_ah, total_wh = [], 0.0, 0.0, 0.0
        for item in profile:
            label, twr, dur = item if len(item) == 3 else (f"seg{len(segs)+1}", *item)
            s = self.segment(twr, dur)
            s["label"] = label
            segs.append(s)
            total_s += dur
            total_ah += s["charge_Ah"]
            total_wh += s["energy_Wh"]

        usable_ah = self.capacity_ah * self.usable_fraction
        result = dict(segments=segs, total_time_s=total_s, total_charge_Ah=total_ah,
                        total_energy_Wh=total_wh, average_current_A=total_ah * 3600 / total_s,
                        capacity_used_pct=100 * total_ah / self.capacity_ah,
                        usable_remaining_Ah=usable_ah - total_ah,
                        mission_ok=total_ah <= usable_ah)

        if verbose:
            print(f"{'segment':<14}{'TWR':>6}{'t [s]':>8}{'thr %':>7}{'RPM':>7}"
                    f"{'I_batt [A]':>11}{'I_esc [A]':>9}{'I_motor [A]':>9}{'Ah':>9}")
            for s in segs:
                flag = "  (!) outside prop table" if s["out_of_prop_table"] else ""
                print(f"{s['label']:<14}{s['twr']:>6.2f}{s['duration_s']:>8.0f}"
                        f"{s['throttle']*100:>7.1f}{s['rpm']:>7.0f}"
                        f"{s['battery_current_A']:>11.2f}{s['esc_input_current_A']:>9.2f}"
                        f"{s['motor_current_A']:>9.2f}{s['charge_Ah']:>9.4f}{flag}")

            print("-" * 63)
            print(f"Total time {total_s:.0f} s | charge {total_ah:.3f} Ah "
                    f"({result['capacity_used_pct']:.0f}% of {self.capacity_ah} Ah; "
                    f"limit {self.usable_fraction*100:.0f}%) | avg {result['average_current_A']:.2f} A")
            print("Mission OK" if result["mission_ok"] else "MISSION EXCEEDS USABLE CAPACITY",
                    f"| usable margin {result['usable_remaining_Ah']:.3f} Ah")
        return result

    def endurance_s(self, twr: float = 1.0) -> float:
        """Seconds you could hold a constant TWR on the usable capacity."""
        seconds = self.capacity_ah * self.usable_fraction * 3600.0 / self.operating_point(twr)["battery_current_A"]
        print(f'Endurance at TWR {twr:.2f}: {seconds:.1f} s ({seconds/60:.1f} min)')
        pass