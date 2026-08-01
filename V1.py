import math
import os
import matplotlib.pyplot as plt

#initials
rho = 1.225             # [kg/m^3]
mass = 800  # [kg]
g0 = 9.81   # [m/s^2]

#car info
engine_force = 12000    # [N]
tiregrip_mu = 1.8
Cd = 0.7
A = 1.5                 # [m^2]

#steps
dx = 0.5                # [m]

#tracks info, angle measured in degrees
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

#physics functions to use in main sim
def drag_force(velocity):
    return 0.5*rho*(velocity**2)*Cd*A
def corner_speed(radius):
    return math.sqrt(tiregrip_mu*radius*g0)

#make functions defining straight vs corner segments
def simulate_straight(category, entry_speed, target_speed):
    length = category["length"]
    time = 0
    position = 0
    velocity = entry_speed
    braking = False
    brake_point = None
    speed_profile = []

    c = tiregrip_mu * mass * g0 / (0.5 * rho * Cd * A)

    while position < length:
        remaining = length - position
        allowed_velocity = math.sqrt((c + target_speed ** 2) * math.exp(2 * 0.5 * rho * Cd * A * remaining / mass) - c)

        was_braking = braking
        braking = velocity >= allowed_velocity
        if braking and not was_braking and brake_point is None:
            brake_point = position

        drag = drag_force(velocity)
        force = engine_force - drag
        if braking:
            force = -(tiregrip_mu*mass*g0) - drag

        acceleration = force / mass
        v_new = math.sqrt(max(velocity ** 2 + 2 * acceleration * dx, 0))
        time += dx / ((velocity + v_new) / 2)
        velocity = v_new
        position += dx
        speed_profile.append((position, velocity))
    return velocity, time, brake_point, speed_profile

def simulate_corner(category, entry_speed):
    radius = category["radius"]
    angle = category["angle"]
    speed = corner_speed(radius)
    length = radius*math.radians(angle)
    time = length/speed
    profile = [(0, speed), (length, speed)]
    return speed, time, profile

segment_results, speed_profile = [], []

velocity = 0
total_time = 0
lap_distance = 0
for i in range(len(track)):
    current = track[i]
    if current["type"] == "straight":
        next_segment = track[i+1]
        target_speed = corner_speed(next_segment["radius"])
        velocity, dt, brake_point, segment_profile = simulate_straight(current, velocity, target_speed)
        total_time += dt
        segment_results.append({'segment': i+1,'type': 'straight','time': dt,'exit_speed': velocity,'brake_point': brake_point})
        speed_profile.extend((lap_distance + position, velocity) for position, velocity in segment_profile)
        lap_distance += current["length"]
    elif current["type"] == "corner":
        velocity, dt, segment_profile = simulate_corner(current, velocity)
        total_time += dt
        segment_results.append({'segment': i+1,'type': 'corner','time': dt,'exit_speed': velocity})
        speed_profile.extend((lap_distance + position, velocity) for position, velocity in segment_profile)
        lap_distance += current["radius"] * math.radians(current["angle"])

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
plt.title(f"Speed vs Distance")
plt.grid(True, alpha=0.3)
os.makedirs("figures", exist_ok=True)
plt.savefig("figures/speed_profile.png", dpi=150, bbox_inches="tight")
plt.show()

print("Lap Summary:")
for segment in segment_results:
    print(f"Segment {segment['segment']} | {segment['type'].capitalize():8} | t={segment['time']:6.3f}s | Exit Speed={segment['exit_speed']:.1f} m/s")
print(f"\nTotal Lap Time: {total_time:.3f} s")
