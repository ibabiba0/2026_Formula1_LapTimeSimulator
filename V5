import math
import os
import matplotlib.pyplot as plt

#constants
rho = 1.225             # [kg/m^3]
g0 = 9.81               # [m/s^2]

#car info
mass = 800              # [kg]
redline = 15000         # [rpm]
tiregrip_mu = 1.8
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
    0.90]
final_drive_ratio = 3.60
shift_time = 0.04       # [s], time with no drive during a gearshift

#steps
dx = 0.5                # [m]

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

#physics functions
def drag_force(velocity):
    return 0.5*rho*(velocity**2)*Cd*A

def corner_speed(radius):
    return math.sqrt((tiregrip_mu*mass*g0)/((mass/radius)-(0.5*tiregrip_mu*rho*Cl*A)))

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

#segment simulators
def simulate_straight(category, entry_speed, target_speed):
    length = category["length"]
    time = 0
    position = 0
    velocity = entry_speed
    braking = False
    brake_point = None
    speed_profile = []
    rpm_profile = []

    # pick a starting gear that suits the entry speed
    gear = 0
    shift_timer = 0.0       # counts down remaining shift time
    wheel_rpm = (entry_speed * 60) / wheel_circumference
    while gear < len(gear_ratios) - 1 and wheel_rpm * gear_ratios[gear] * final_drive_ratio > redline:
        gear += 1

    while position < length:
        # braking constant - recomputed each step because downforce depends on velocity
        c = tiregrip_mu * ((mass * g0) + downforce(velocity)) / (0.5 * rho * Cd * A)

        remaining = length - position
        allowed_velocity = math.sqrt((c + target_speed ** 2) * math.exp(2 * 0.5 * rho * Cd * A * remaining / mass) - c)

        was_braking = braking
        braking = velocity >= allowed_velocity
        if braking and not was_braking and brake_point is None:
            brake_point = position

        traction_limit = tiregrip_mu * (mass * g0 + downforce(velocity))
        if velocity < 1:
            engine_force = traction_limit          # off the line: grip-limited, avoids /0
        else:
            # drivetrain: vehicle speed -> wheel rpm -> engine rpm
            wheel_rpm = (velocity * 60) / wheel_circumference
            engine_rpm = wheel_rpm * gear_ratios[gear] * final_drive_ratio

            # shift up at redline - starts the shift timer
            if engine_rpm >= redline and gear < len(gear_ratios) - 1 and shift_timer <= 0:
                gear += 1
                engine_rpm = wheel_rpm * gear_ratios[gear] * final_drive_ratio
                shift_timer = shift_time

            # engine model: rpm -> torque -> wheel torque -> force
            if shift_timer > 0:
                engine_force = 0.0          # clutch out mid-shift, no drive
            else:
                torque = engine_torque(engine_rpm)
                wheel_torque = torque * gear_ratios[gear] * final_drive_ratio
                engine_force = min(traction_limit, wheel_torque / wheel_radius)

            rpm_profile.append((position, engine_rpm, gear))

        drag = drag_force(velocity)
        force = engine_force - drag

        if braking:
            max_braking_force = tiregrip_mu * (mass * g0 + downforce(velocity))
            force = -max_braking_force - drag

        acceleration = force / mass
        v_new = math.sqrt(max(velocity ** 2 + 2 * acceleration * dx, 0))
        step_time = dx / ((velocity + v_new) / 2)
        time += step_time
        if shift_timer > 0:
            shift_timer -= step_time
        velocity = v_new
        position += dx
        speed_profile.append((position, velocity))
    return velocity, time, brake_point, speed_profile, rpm_profile

def simulate_corner(category, entry_speed):
    radius = category["radius"]
    angle = category["angle"]
    speed = corner_speed(radius)
    length = radius*math.radians(angle)     # s = r*theta
    time = length/speed
    profile = [(0, speed), (length, speed)]
    return speed, time, profile

#main loop
segment_results, speed_profile, rpm_profile = [], [], []
velocity = 0
total_time = 0
lap_distance = 0
for i in range(len(track)):
    current = track[i]
    if current["type"] == "straight":
        next_segment = track[i+1]
        target_speed = corner_speed(next_segment["radius"])
        velocity, dt, brake_point, segment_profile, segment_rpm = simulate_straight(current, velocity, target_speed)
        total_time += dt
        segment_results.append({'segment': i+1, 'type': 'straight', 'time': dt, 'exit_speed': velocity, 'brake_point': brake_point})
        speed_profile.extend((lap_distance + position, velocity) for position, velocity in segment_profile)
        rpm_profile.extend((lap_distance + p, r, g) for p, r, g in segment_rpm)
        lap_distance += current["length"]
    elif current["type"] == "corner":
        velocity, dt, segment_profile = simulate_corner(current, velocity)
        total_time += dt
        segment_results.append({'segment': i+1, 'type': 'corner', 'time': dt, 'exit_speed': velocity})
        speed_profile.extend((lap_distance + position, velocity) for position, velocity in segment_profile)
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

#summary
print("Lap Summary:")
for segment in segment_results:
    print(f"Segment {segment['segment']} | {segment['type'].capitalize():8} | t={segment['time']:6.3f}s | Exit Speed={segment['exit_speed']:.1f} m/s")
print(f"\nTotal Lap Time: {total_time:.3f} s")
