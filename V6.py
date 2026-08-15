import math
import os
import matplotlib.pyplot as plt

#constants
rho = 1.225             # [kg/m^3]
g0 = 9.81               # [m/s^2]

ENABLE_ELLIPSE         = True
ENABLE_LOAD_SENS       = True
ENABLE_WEIGHT_TRANSFER = True

#car info
mass = 800              # [kg]
redline = 15000         # [rpm]
Cd = 0.7
Cl = 3.0
A = 1.5                 # [m^2]
wheel_radius = 0.36     # [m]
wheel_circumference = math.pi*2*wheel_radius
gear_ratios = [
    2.60,
    2.05,
    1.72,
    1.48,
    1.29,
    1.13,
    1.00,
    0.90
]
final_drive_ratio = 7.00

#tire info
tiregrip_mu = 1.8
k_load = 0.15   #load sensitivity exponent
Fz_ref = mass*g0 / 4    # weight distributed equally amongst four ties

# chassis
height_cg     = 0.30    # [m]  CG: center of gravity, height
wheelbase     = 3.60    # [m]
track_width   = 1.60    # [m]
weight_dist_f = 0.45    # fraction of static weight on front axle
aero_balance  = 0.45    # fraction of downforce on front axle
roll_share_f  = 0.50    # fraction of lateral transfer taken by front axle

#steps
dx = 0.5                # [m]
shift_time = 0.04       # [s]
V_TOP = 100.0           # [m/s] cap for the corner speed solver
corner_transition = 25.0    # [m]

#track info, angle measured in degrees
track = [
    {"type": "straight", "length": 1000},
    {"type": "corner", "radius": 100, "angle": 90},
    {"type": "straight", "length": 450},
    {"type": "corner", "radius": 40, "angle": 180},
    {"type": "straight", "length": 650},
    {"type": "corner", "radius": 120, "angle": 90},
    {"type": "straight", "length": 300},
    {"type": "corner", "radius": 60, "angle": 90}
]

def mu(Fz): # Load sensitive friction for INDIVIDUAL tires
    if Fz <= 1.0:
        return 0.0
    if not ENABLE_LOAD_SENS:
        return tiregrip_mu
    return tiregrip_mu * (Fz / Fz_ref) ** (-k_load)


def tire_loads(velocity, ax, ay):   # s.t. ax : longitudinal acceleration, ay : lateral acceleration
    Fz_total = mass * g0 + downforce(velocity)

    if not ENABLE_WEIGHT_TRANSFER:
        return [Fz_total / 4.0] * 4

   # how the axle loads split between rear and front, then split to longitudinal load transfer
    fz_frontaxle_load = weight_dist_f * mass * g0 + aero_balance * downforce(velocity)
    fz_rearaxle_load = Fz_total - fz_frontaxle_load
    dFx = mass * ax * height_cg / wheelbase     # dFx: longitudinal load transfer
    fz_frontaxle_load -= dFx
    fz_rearaxle_load += dFx

    # define lateral transfer
    total_lateral_load_transfer = mass * abs(ay) * height_cg / track_width
    front_lateral_load_transfer, rear_lateral_load_transfer = roll_share_f * total_lateral_load_transfer, (1.0 - roll_share_f) * total_lateral_load_transfer

    loads = [fz_frontaxle_load / 2 - front_lateral_load_transfer, fz_frontaxle_load / 2 + front_lateral_load_transfer, fz_rearaxle_load / 2 - rear_lateral_load_transfer, fz_rearaxle_load / 2 + rear_lateral_load_transfer]
    return [max(f, 0.0) for f in loads]  # clamp = wheel lift

def grip_capacity(velocity, ax, ay):   # Total friction force the four tires can generate [N]
    return sum(mu(Fz) * Fz for Fz in tire_loads(velocity, ax, ay))

def longitudinal_grip(velocity, ax, ay):
    grip = grip_capacity(velocity, ax, ay)
    if grip <= 0.0:
        return 0.0
    if not ENABLE_ELLIPSE:
        return grip
    usage = min(mass * abs(ay) / grip, 1.0)
    return grip * math.sqrt(max(1.0 - usage ** 2, 0.0))

def braking_grip(velocity, kappa=0.0):
    ay = kappa * velocity ** 2
    ax = 0.0
    grip = longitudinal_grip(velocity, ax, ay)
    for _ in range(3):
        ax = -(grip + drag_force(velocity)) / mass
        grip = longitudinal_grip(velocity, ax, ay)
    return grip

def drag_force(velocity):
    return 0.5*rho*(velocity**2)*Cd*A

def corner_speed(radius):
    def residual(v):
        return grip_capacity(v, 0.0, v**2 / radius) - mass * v**2 / radius

    if residual(V_TOP) > 0:
        return V_TOP
    lo, hi = 0.5, V_TOP
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if residual(mid) > 0:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)

def downforce(velocity):
    return 0.5*rho*Cl*A*(velocity**2)

def engine_torque(rpm):
    if rpm < 3000:
        return 300
    elif rpm < 9000:
        return 500
    elif rpm < 15000:
        return 500 - (rpm - 9000) * 0.03
    else:
        return 0

def curvature_at(position, length, next_radius):
    if next_radius is None or corner_transition <= 0:
        return 0.0
    remaining = length - position
    if remaining >= corner_transition:
        return 0.0
    return (1.0 - remaining / corner_transition) / next_radius


def braking_profile(length, target_speed, next_radius):
    n = int(round(length / dx))
    v_allowed = [0.0] * (n + 1)
    v_allowed[n] = target_speed
    for i in range(n - 1, -1, -1):
        v = v_allowed[i + 1]
        kappa = curvature_at(i * dx, length, next_radius)
        decel = (braking_grip(v, kappa) + drag_force(v)) / mass
        v_allowed[i] = math.sqrt(v ** 2 + 2 * decel * dx)
    return v_allowed

def simulate_straight(category, entry_speed, target_speed, next_radius=None):
    length = category["length"]
    v_allowed = braking_profile(length, target_speed, next_radius)
    time = 0
    position = 0
    velocity = entry_speed
    braking = False
    brake_point = None
    speed_profile = []
    rpm_profile = []
    load_profile = []
    gear = 0
    shift_timer = 0.0       # counts down remaining shift time
    ax_prev = 0.0           # last step's longitudinal accel, feeds weight transfer
    wheel_rpm = (entry_speed * 60) / wheel_circumference
    while gear < len(gear_ratios) - 1 and wheel_rpm * gear_ratios[gear] * final_drive_ratio > redline:
        gear += 1

    while position < length:
        ay = curvature_at(position, length, next_radius) * velocity ** 2
        traction_limit = longitudinal_grip(velocity, ax_prev, ay)

        brake_grip = braking_grip(velocity, curvature_at(position, length, next_radius))
        allowed_velocity = v_allowed[min(int(round(position / dx)), len(v_allowed) - 1)]

        was_braking = braking
        braking = velocity >= allowed_velocity
        if braking and not was_braking and brake_point is None:
            brake_point = position

        if velocity < 1:
            engine_force = traction_limit          # off the line: grip-limited, avoids /0
        else:
            wheel_rpm = (velocity * 60) / wheel_circumference
            engine_rpm = wheel_rpm * gear_ratios[gear] * final_drive_ratio

            if engine_rpm >= redline and gear < len(gear_ratios) - 1 and shift_timer <= 0:
                gear += 1
                engine_rpm = wheel_rpm * gear_ratios[gear] * final_drive_ratio
                shift_timer = shift_time

            if shift_timer > 0:
                engine_force = 0.0
            else:
                torque = engine_torque(engine_rpm)
                wheel_torque = torque * gear_ratios[gear] * final_drive_ratio
                engine_force = min(traction_limit, wheel_torque / wheel_radius) # engine_force = wheel_torque/wheel_radius

            rpm_profile.append((position, engine_rpm, gear))

        drag = drag_force(velocity)
        force = engine_force - drag

        if braking:
            max_braking_force = brake_grip
            force = -max_braking_force - drag

        acceleration = force / mass
        v_new = math.sqrt(max(velocity ** 2 + 2 * acceleration * dx, 0))
        step_time = dx / ((velocity + v_new) / 2)
        time += step_time
        if shift_timer > 0:
            shift_timer -= step_time
        ax_prev = acceleration
        load_profile.append((position, tire_loads(velocity, acceleration, ay)))
        velocity = v_new
        position += dx
        speed_profile.append((position, velocity))
    return velocity, time, brake_point, speed_profile, rpm_profile, load_profile

def simulate_corner(category, entry_speed):
    radius = category["radius"]
    angle = category["angle"]
    speed = corner_speed(radius)
    length = radius*math.radians(angle)     # s = r*theta
    time = length/speed
    profile = [(0, speed), (length, speed)]
    ay = speed**2 / radius
    loads = tire_loads(speed, 0.0, ay)
    load_profile = [(0, loads), (length, loads)]
    return speed, time, profile, load_profile

#main loop
segment_results, speed_profile, rpm_profile, load_profile = [], [], [], []
velocity = 0
total_time = 0
lap_distance = 0
for i in range(len(track)):
    current = track[i]
    if current["type"] == "straight":
        next_segment = track[i+1]
        target_speed = corner_speed(next_segment["radius"])
        velocity, dt, brake_point, segment_profile, segment_rpm, segment_loads = simulate_straight(current, velocity, target_speed, next_segment["radius"])
        total_time += dt
        segment_results.append({'segment': i+1, 'type': 'straight', 'time': dt, 'exit_speed': velocity, 'brake_point': brake_point})
        speed_profile.extend((lap_distance + position, velocity) for position, velocity in segment_profile)
        rpm_profile.extend((lap_distance + p, r, g) for p, r, g in segment_rpm)
        load_profile.extend((lap_distance + p, l) for p, l in segment_loads)
        lap_distance += current["length"]
    elif current["type"] == "corner":
        velocity, dt, segment_profile, segment_loads = simulate_corner(current, velocity)
        total_time += dt
        segment_results.append({'segment': i+1, 'type': 'corner', 'time': dt, 'exit_speed': velocity})
        speed_profile.extend((lap_distance + position, velocity) for position, velocity in segment_profile)
        load_profile.extend((lap_distance + p, l) for p, l in segment_loads)
        lap_distance += current["radius"] * math.radians(current["angle"])

os.makedirs("figures", exist_ok=True)

def save_figure(name):
    try:
        plt.savefig(os.path.join("figures", name), dpi=150)
    except OSError as err:
        print(f"[warn] could not save {name}: {err}")

#plot 1: speed vs distance
distances, speeds = zip(*speed_profile)
plt.figure(figsize=(12, 5))
plt.plot(distances, speeds, linewidth=1.3)
d = 0
for seg in track:
    if seg["type"] == "corner":
        seg_len = seg["radius"] * math.radians(seg["angle"])
        plt.axvspan(d, d + seg_len, color="grey", alpha=0.15)
        d += seg_len
    else:
        d += seg["length"]
plt.xlabel("Distance (m)")
plt.ylabel("Speed (m/s)")
plt.title("Speed vs Distance")
plt.grid(True, alpha=0.3)
save_figure("speed_profile.png")
plt.show()

#plot 2: engine torque and power vs rpm (engine property, no sim needed)
rpms = list(range(1000, redline + 1, 100))
torques = [engine_torque(r) for r in rpms]
powers = [engine_torque(r) * r * 2 * math.pi / 60 / 1000 for r in rpms]   # [kW]
fig, ax1 = plt.subplots(figsize=(9, 5))
ax1.plot(rpms, torques, color="tab:blue", label="Torque")
ax1.set_xlabel("Engine RPM")
ax1.set_ylabel("Torque (N·m)", color="tab:blue")
ax1.grid(True, alpha=0.3)
ax2 = ax1.twinx()
ax2.plot(rpms, powers, color="tab:red", label="Power")
ax2.set_ylabel("Power (kW)", color="tab:red")
plt.title("Engine Torque and Power vs RPM")
save_figure("engine_curve.png")
plt.show()

#plot 3: rpm and gear vs distance
rd, rr, rg = zip(*rpm_profile)
fig, (axa, axb) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
axa.plot(rd, rr, linewidth=1.0)
axa.axhline(redline, color="red", linestyle="--", linewidth=0.8, label="redline")
axa.set_ylabel("Engine RPM")
axa.legend()
axa.grid(True, alpha=0.3)
axb.step(rd, [g + 1 for g in rg], where="post", linewidth=1.2)
axb.set_ylabel("Gear")
axb.set_xlabel("Distance (m)")
axb.grid(True, alpha=0.3)
plt.suptitle("Engine RPM and Gear vs Distance")
save_figure("rpm_gear.png")
plt.show()

#plot 4: tire vertical loads vs distance  (new in V6)
ld, ll = zip(*load_profile)
plt.figure(figsize=(12, 4))
for j, name in enumerate(["FL", "FR", "RL", "RR"]):
    plt.plot(ld, [l[j] for l in ll], linewidth=0.9, label=name)
plt.xlabel("Distance (m)")
plt.ylabel("Vertical load (N)")
plt.title("Tire Vertical Loads vs Distance")
plt.legend()
plt.grid(True, alpha=0.3)
save_figure("tire_loads.png")
plt.show()

#summary
print("\nCorner speeds:")
for seg in track:
    if seg["type"] == "corner":
        print(f"  R={seg['radius']:3d} m -> {corner_speed(seg['radius']):.1f} m/s")
print("\nLap Summary:")
for segment in segment_results:
    print(f"Segment {segment['segment']} | {segment['type'].capitalize():8} | t={segment['time']:6.3f}s | Exit Speed={segment['exit_speed']:.1f} m/s")
print(f"\nTotal Lap Time: {total_time:.3f} s")
