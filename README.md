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
- `speed profile` giving (distance, velocity) samples across the lap, ready to plot (as seen in figures/v1)

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
