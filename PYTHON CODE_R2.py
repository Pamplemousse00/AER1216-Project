import math
from dataclasses import dataclass
from typing import Tuple

# -----------------------
# Constants & atmosphere
# -----------------------

G = 9.80665  # m/s^2
R = 287.05   # J/(kg*K)
T0 = 288.15  # K
P0 = 101325  # Pa
L = 0.0065   # K/m (temperature lapse rate)
RHO0 = 1.225 # kg/m^3 at sea level


@dataclass
class Aircraft:
    mass_kg: float           # takeoff mass (MTOW or operating mass)
    wing_area_m2: float
    CD0: float               # zero-lift drag coefficient
    k: float                 # induced drag fuavtor
    CL_max: float            # max lift coefficient
    battery_capacity_Wh: float
    prop_efficiency: float   # propulsive efficiency (0-1)
    P_max_W: float           # max shaft power at sea level (motor)


def isa_density(altitude_m: float) -> float:
    """Very simple ISA density model up to ~11 km."""
    T = T0 - L * altitude_m
    P = P0 * (T / T0) ** (G / (R * L))
    rho = P / (R * T)
    return rho


def weight_Newtons(uav: Aircraft) -> float:
    return uav.mass_kg * G


# -----------------------
# Basic aero & drag
# -----------------------


def drag_components(uav: Aircraft, rho: float, V: float) -> Tuple[float, float, float]:
    W = weight_Newtons(uav)
    A = uav.wing_area_m2
    CL = W / (0.5 * rho * V**2 * A)
    CD = uav.CD0 + uav.k * CL**2
    D = 0.5 * rho * V**2 * A * CD
    return CL, CD, D


def power_required_W(uav: Aircraft, rho: float, V: float) -> float:
    _, _, D = drag_components(uav, rho, V)
    return D * V


def power_available_W(uav: Aircraft, rho: float) -> float:
    return uav.P_max_W * uav.prop_efficiency


def stall_speed(uav: Aircraft, rho: float) -> float:
    W = weight_Newtons(uav)
    return math.sqrt(2 * W / (rho * uav.wing_area_m2 * uav.CL_max))




def range_endurance_electric(uav: Aircraft,
                             P_cruise_W: float,
                             V_cruise: float) -> Tuple[float, float]:
    usable_energy_J = uav.battery_capacity_Wh * 3600 * uav.prop_efficiency
    endurance_s = usable_energy_J / P_cruise_W
    range_m = endurance_s * V_cruise
    return range_m, endurance_s



def min_max_speed_at_altitude(uav: Aircraft,
                              altitude_m: float,
                              V_max_guess: float) -> Tuple[float, float]:
    
    # V_max_guess is an upper bound to search up to e.g. 60 m/s

    rho = isa_density(altitude_m)
    V_min = stall_speed(uav, rho)

    P_avail = power_available_W(uav, rho)
    V = V_min * 1.05  # start just above stall
    V_step = 0.5      # m/s step
    V_max = V_min
    while V <= V_max_guess:
        P_req = power_required_W(uav, rho, V)
        if P_req <= P_avail:
            V_max = V
        else:
            break
        V += V_step

    return V_min, V_max


def best_L_over_D_speed(uav: Aircraft, altitude_m: float) -> float:

    # Occurs when induced drag = parasite drag: k * CL^2 = CD0
    rho = isa_density(altitude_m)
    W = weight_Newtons(uav)
    S = uav.wing_area_m2
    CL_md = math.sqrt(uav.CD0 / uav.k)
    V_md = math.sqrt(2 * W / (rho * S * CL_md))
    return V_md


def best_endurance_speed(uav: Aircraft, altitude_m: float) -> float:
    # Occurs when induced drag = 3 * CD0.
    rho = isa_density(altitude_m)
    W = weight_Newtons(uav)
    S = uav.wing_area_m2
    CL_me = math.sqrt(3.0 * uav.CD0 / uav.k)
    V_me = math.sqrt(2 * W / (rho * S * CL_me))
    return V_me



def climb_rate(uav: Aircraft, altitude_m: float, V: float) -> float:

    # ROC = (P_avail - P_req) / W

    rho = isa_density(altitude_m)
    P_avail = power_available_W(uav, rho)
    P_req = power_required_W(uav, rho, V)
    W = weight_Newtons(uav)
    ROC = (P_avail - P_req) / W
    max_ROC = uav.prop_efficiency*P_avail/W + (0.5*rho*V_max_100*uav.CD0/(W/uav.wing_area_m2))-(2*uav.k*(W/uav.wing_area_m2)/(rho*V_max_100))
    if ROC < 0:
      print("Warning: Negative climb rate (descent required).")
    return ROC, max_ROC


def frange(start: float, stop: float, step: float):
    x = start
    while x <= stop:
        yield x
        x += step


def estimate_ceiling(uav: Aircraft,
                     roc_service_mps: float = 0.5,
                     h_max_search_m: float = 6000.0,
                     dh_m: float = 100.0) -> Tuple[float, float]:

    # Service ceiling: altitude where max ROC falls to 0.5m/s
    # Absolute ceiling: altitude where max ROC = 0.

    service = None
    absolute = None

    W = weight_Newtons(uav)
    for h in [i for i in frange(0.0, h_max_search_m, dh_m)]:
        rho = isa_density(h)
        # search over speeds between 1.1*stall and some reasonable upper bound
        V_stall = stall_speed(uav, rho)
        V_lo = 1.1 * V_stall
        V_hi = 60.0  # upper speed bound to search (adjust as needed)
        V_step = 1.0
        max_roc = 0.0
        V = V_lo
        while V <= V_hi:
            P_req = power_required_W(uav, rho, V)
            P_avail = power_available_W(uav, rho)
            roc = (P_avail - P_req) / W
            if roc > max_roc:
                max_roc = roc
            V += V_step

        if max_roc <= 0 and absolute is None:
            absolute = h
            break
        if max_roc <= roc_service_mps and service is None:
            service = h

    return service, absolute


# -----------------------
# Turning performance
# -----------------------

def turn_radius_and_speed(uav: Aircraft,
                          altitude_m: float,
                          bank_angle_deg: float) -> Tuple[float, float, float]:
    """Compute minimum turn radius for a given bank angle.

    We assume a level coordinated turn at the stall speed for that bank.
    Returns (R_min_m, V_turn_mps, load_fuavtor_n).
    """
    rho = isa_density(altitude_m)
    phi = math.radians(bank_angle_deg)
    n = 1.0 / math.cos(phi)  # load fuavtor in level turn

    # Stall speed in a bank: V_stall_phi = V_stall_0 * sqrt(n)
    V_stall_0 = stall_speed(uav, rho)
    V_turn = V_stall_0 * math.sqrt(n)

    R_min = V_turn**2 / (G * math.tan(phi))
    return R_min, V_turn, n



if __name__ == "__main__":
    #calculating inputs
    bat_mAh = 5000  # mAh
    bat_V = 22.2 #V - 6S LiPo
    bat_capacity_A = (bat_mAh / 1000) * bat_V  # Wh

    AR = 9 # Aspect Ratio
    e = 0.7  # Oswald efficiency factor for pure rectangle wing
    k_rect = 1 / (math.pi * AR * e) # induced drag factor

    #defining inputs
    uav = Aircraft(
        mass_kg=5.0,            # 10 kg MTOW
        wing_area_m2=0.613125,        # m^2
        CD0=0.006,
        k=k_rect,                 # induced drag factor 1/(pi*AR*e)
        CL_max=1.67,
        battery_capacity_Wh=bat_capacity_A,  
        prop_efficiency=0.7,
        P_max_W=2770.0          # 1.5 kW motor
    )

    # (a) & (b) Range and endurance at some cruise
    h_cruise = 100.0  # m
    V_cruise = best_L_over_D_speed(uav, h_cruise)  # (f) ideal/designed cruise velocity
    rho_cruise = isa_density(h_cruise)
    P_req_cruise = power_required_W(uav, rho_cruise, V_cruise)
    R_m, E_s = range_endurance_electric(uav, P_req_cruise, V_cruise)

    # (c) & (d) Power available/required at cruise
    P_avail_cruise = power_available_W(uav, rho_cruise)

    # (e) Minimum & maximum airspeed at 100 m
    V_min_100, V_max_100 = min_max_speed_at_altitude(uav, 100.0, V_max_guess=60.0)

    # (g) Stall speed at sea level
    V_stall_sea = stall_speed(uav, RHO0)

    # (h) Service & absolute ceiling
    h_service, h_absolute = estimate_ceiling(uav)

    # (i) Maximum climb rate near sea level at best endurance speed
    V_best_endurance = best_endurance_speed(uav, 0.0)
    roc_max, New_ROC = climb_rate(uav, 0.0, V_best_endurance)

    # (j) Minimum turn radius and associated velocity (e.g., 45° bank at 100 m)
    R_min_turn, V_turn, n_load = turn_radius_and_speed(uav, 100.0, bank_angle_deg=45.0)


    print(f"Cruise power required: {P_req_cruise:.0f} W")
    print(f"Cruise power available: {P_avail_cruise:.0f} W")
    print(f"Range: {R_m/1000:.1f} km, Endurance: {E_s/60:.1f} min")
    print(f"V_min@100m: {V_min_100:.1f} m/s, V_max@100m: {V_max_100:.1f} m/s")
    print(f"Stall (sea level): {V_stall_sea:.1f} m/s")
    print(f"Service ceiling: {h_service} m, Absolute ceiling: {h_absolute} m")
    print(f"Max climb rate: {roc_max:.2f} m/s or {New_ROC} new value at V = {V_best_endurance:.1f} m/s")
    print(f"Min turn radius: {R_min_turn:.1f} m at V = {V_turn:.1f} m/s, n = {n_load:.2f}")
