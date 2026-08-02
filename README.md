# Formula 1 Quasi-Steady-State Lap Time Simulator
The objective of this project is to develop a quasi-steady-state (QSS) lap time simulator that calculates how long a given car takes to complete a given track. The project explores the fundamental physics involved in real Formula 1 and motorsport engineering.

## V1
Given a car and a simple track consisting of basic straights and corners, the initial version of the lap time simulation uses fundamental physics in real motorsports to produce the total lap time and a per-segment breakdown and speed trace.

### Inputs
**Car Specs and Natural Constants**
| Parameter | Value | Units |
|-----------|-------|-------|
| Mass | 800 | kg |
| Engine force | 12000 | N |
| Tire grip (μ) | 1.8 | — |
| C_d | 0.7 | — |
| Frontal area (A) | 1.5 | m² |
| Air density (ρ) | 1.225 | kg/m³ |

**Track Map:**
| # | Type | Length | Radius | Angle |
|---|------|--------|--------|-------|
| 1 | Straight | 1000 m | — | — |
| 2 | Corner | — | 100 m | 90° |
| 3 | Straight | 450 m | — | — |
| 4 | Corner | — | 40 m | 180° |
| 5 | Straight | 650 m | — | — |
| 6 | Corner | — | 120 m | 90° |
| 7 | Straight | 300 m | — | — |
| 8 | Corner | — | 60 m | 90° |

### Physics
- **Tire-limited corner speeds**
  
$$F_{\text{centripetal}} = F_{\text{tires}}$$

$$\frac{mv^2}{r} = \mu_{\text{tire grip}}mg$$

  The centripetal force required to keep the car on its circular path is supplied entirely by the tires. This tire force is essentially static friction, with the coefficient of static friction set by the tire's grip. Setting the maximum available friction equal to the required centripetal force gives the maximum corner speed (code variable: `target_velocity`)
  
$$v = \sqrt{\mu g r}$$
-  **Drag**
  
  Drag is the resistive force opposing the car's forward motion. This version maintains a fixed drag coefficient such that the drag scales purely with speed.

$$F_{\text{drag}} = \tfrac{1}{2}\rho C_d A v^2$$

- **Engine Acceleration**
  
  Based on Newton's second Law:

$$a = F/m$$

  Incorporating the drag force makes net force equal engine force minus drag:

$$F_{\text{drag}} = \tfrac{1}{2}\rho C_d A v^2$$
  
- **Braking to achieve entry speed at corner**
  
  With the simple track configuration, it is given that before every corner will be a straight track. The code calculates the maximum allowed speed to meet the target velocity while entering the curve, as mentioned previously. On a straight track, the car is going much faster than the target entry velocity. The code calculates the fastest the car could be going while still being able to decelerate over the remaining distance to meet the target velocity.

### Outputs
- Per-segment: time, exit speed, braking point
- Total lap time
- `speed profile` giving (distance, velocity) samples across the lap
- Plot of speed vs distance: as seen in '''figures/v1_output'''

```Lap Summary:
Segment 1 | Straight | t=14.284s | Exit Speed=42.2 m/s
Segment 2 | Corner   | t= 3.738s | Exit Speed=42.0 m/s
Segment 3 | Straight | t= 7.185s | Exit Speed=26.9 m/s
Segment 4 | Corner   | t= 4.728s | Exit Speed=26.6 m/s
Segment 5 | Straight | t= 9.354s | Exit Speed=46.2 m/s
Segment 6 | Corner   | t= 4.095s | Exit Speed=46.0 m/s
Segment 7 | Straight | t= 5.045s | Exit Speed=32.8 m/s
Segment 8 | Corner   | t= 2.896s | Exit Speed=32.5 m/s

Total Lap Time: 51.325 s
```

## V2
Incorporate downforce into the simulation.

### Physics:
- **Downforce**

Downforce is one of the primary aerodynamic forces acting on a motorvehicle while in motion. Inverted wings and other aerodynamic devices on a car generate a downward force pushing the car into the track. This additional force increases the normal force acting on the tires, allowing them to generate greater frictional forces and therefore achieve higher braking and cornering performance.

The Downforce Equation:

$$F_L = \frac{1}{2} \rho C_L A v^2$$

🤔 **What changes in the code:**
| Parameter | Previous | with Downforce |
|-----------|-------|-------|
| Downforce | — | ```0.5*rho*Cl*A*(velocity**2)``` |
| Corner Speed | ```math.sqrt(tiregrip_mu * radius * g0)``` | ```math.sqrt((tiregrip_mu * mass * g0)/((mass/radius)-(0.5 * tiregrip_mu * rho * Cl * A)))``` |
| Braking Force | ```force = -(tiregrip_mu * mass * g0) - drag``` | ``` max_braking_force = (tiregrip_mu * (mass*g0 + downforce(velocity)))``` & ```force = -max_braking_force - drag``` |
| c | ```c = tiregrip_mu * mass * g0 / (0.5 * rho * Cd * A)``` | ```c = tiregrip_mu * ((mass * g0) + downforce(velocity)) / (0.5 * rho * Cd * A)``` |

*Notes on c:* The original analytical braking equation assumed a constant braking force. With the addition of downforce into the equation, which varies with velocity, the braking force constant c is now recalculated every simulation step as an approximation.

### Outputs
```Lap Summary:
Segment 1 | Straight | t=13.213s | Exit Speed=82.8 m/s
Segment 2 | Corner   | t= 2.304s | Exit Speed=68.2 m/s
Segment 3 | Straight | t= 5.116s | Exit Speed=61.5 m/s
Segment 4 | Corner   | t= 4.100s | Exit Speed=30.6 m/s
Segment 5 | Straight | t= 8.364s | Exit Speed=93.7 m/s
Segment 6 | Corner   | t= 2.071s | Exit Speed=91.0 m/s
Segment 7 | Straight | t= 3.093s | Exit Speed=65.7 m/s
Segment 8 | Corner   | t= 2.294s | Exit Speed=41.1 m/s
```

Total Lap Time: 40.554 s

**Reflection**
- The incorporation of downforce physics on the car significantly increased the speed it carried through corners. For instance:
  - Fastest corner (segment 6) with radius 120 m saw a 97.8% increase in exit speed
  - Tightest corner (segment 4) with radius 40 m saw a 15% increase; suggests downforce is more valuable in high-speed corners
  - Total lap time saw a 10.77 s (21%) improvement
- However, the model utilizes a constant engine force and per-step c approximation, making these results idealized. The next model will incorporate more realistic physics, specifically power-limited acceleration.
