# Robot Arm Installation — Project Context

## Overview
A kinetic art installation consisting of **6 robot arms** mounted in a physical space, controlled in real time via TouchDesigner and an Arduino. Each arm moves in spherical coordinates, has a ribbon attached to its end effector, and is equipped with a DMX color fixture. The system supports synchronized choreography, individual control, and wave-based coordinated motion.

---

## Hardware

### Robot Arm Mechanics
- Each arm moves in **spherical coordinates** defined by two motors:
  - **Theta motor** — stepper motor, azimuthal rotation in the **XZ plane** (around Y axis)
    - Range: 0–360° (continuous, wraps)
    - Controlled by **velocity** — converted to **pulses per second** for the stepper driver
    - Formula: `pulse_per_sec = (theta_velocity / 360.0) * pulses_per_revolution`
  - **Phi motor** — 270° servo motor, elevation from Y+ axis
    - Range: **-135° to +135°** (0° = pointing along Y+)
    - Controlled by **angle** (direct servo position)
- A **ribbon** is attached to the end of each arm
  - Can get caught on the robot body near phi ≈ 0 (Y+ axis) at low speed
  - Speed-dependent exclusion cone around Y+ to be implemented

### Coordinate System
```
Y+ = phi 0°  (arm pointing up)
XZ plane = phi 90°
Y- = phi 180° (unreachable, servo limit is ±135°)

Theta: azimuth in XZ plane, 0–360°, wraps continuously
Phi:   elevation from Y+, -135° to +135°
```

### Stepper Motor (Theta)
- Driver: **DM860**
  - Has built-in optocouplers on signal inputs (PUL, DIR, ENA)
  - Optocouplers require isolated signal ground — sharing Arduino ground with 60V ground defeats isolation
- Output from software: **pulse_per_sec** (derived from theta_velocity)
- Default: 20000 pulses per revolution (configurable via `setPulsesPerRevolution()`)

### Servo Motor (Phi)
- 270° servo, powered at **25.2V**
- Control signal: PWM from Arduino
- Output from software: **phi angle** in [-135, 135]

### Homing Sensor
- Powered at **25.2V**
- Signal brought down via voltage divider: 5.6kΩ and 1.1kΩ → ~4.13V output

### DMX Color Fixture
- One RGB fixture per robot (3 channels: R, G, B)
- Currently represented as `led: [r, g, b]` in state dictionaries
- Output via TouchDesigner Art-Net/DMX

---

## Electronics & Power

### Power System (per robot, inside local power box)
- **60V supply** — powers stepper motor driver (DM860)
- **25.2V DC-DC step-down buck converter** (non-isolated) from 60V
  - Since non-isolated: 25.2V ground = 60V ground (same node)
  - Powers servo motor and homing sensor

### Arduino
- **One Arduino controls all 6 robots**
- Connected to robots via ethernet cable to local power/control box
- **Critical grounding rules:**
  - Arduino ground must be **isolated from 60V/25.2V ground**
  - DM860 optocouplers are defeated if grounds are shared
  - Arduino should be powered from an **isolated 5V supply** (not 12V barrel jack)
  - USB connection should use a **USB isolator** during programming
- **Known hardware issue:** Arduino onboard 3.3V regulator destroyed
  - Root cause: shared ground between Arduino and 60V system defeated DM860 optocouplers, stepper switching transients reached Arduino
  - Fix: isolated 5V supply for Arduino, full ground separation from 60V system

---

## Software — TouchDesigner

### Architecture
TouchDesigner is the primary control environment. **Python Extensions** on Base COMPs are used for all logic, providing persistent object-oriented state across frames.

### Node Structure
```
/project1
    /RobotController    Base COMP — RobotControllerEXT extension
    /Robot1             Base COMP — RobotEXT extension
        const_robot     Constant CHOP — receives motor output (theta, phi, pulse_per_sec)
    /Robot2 ... /Robot6 (identical structure)
```

### How Extensions Work in TD
- Each Base COMP has a Python Extension DAT assigned
- The extension class is instantiated automatically when TD loads
- Persistent state lives on the class instance (survives across frames)
- Methods are called from outside via: `op('RobotController').ext.RobotControllerEXT.Update()`
- The `Update()` method is called every frame from a script or Execute DAT

---

## Python Extensions

### `RobotEXT` class (one instance per Robot COMP)
**Role:** State container for one robot — stores position, velocity, limits, and outputs to CHOP.

**State:**
- `theta` — current azimuth [0, 360°)
- `phi` — current elevation [-135°, 135°]
- `theta_velocity` — deg/sec
- `phi_velocity` — deg/sec
- `pulses_per_revolution` — stepper resolution (default 20000)
- `prev_theta_velocity`, `prev_phi_velocity` — for acceleration clamping

**Key limits (configurable):**
- `max_theta_velocity` — 250 deg/sec
- `max_phi_velocity` — 150 deg/sec
- `max_theta_acceleration` — 100 deg/sec²
- `max_phi_acceleration` — 100 deg/sec²

**Key methods:**
```python
SetState(theta, phi, ledMatrix, dt)  # update position + compute velocity/acceleration
PushToCHOP()                          # write theta, phi, pulse_per_sec to const_robot CHOP
GetState()                            # returns dict: theta, phi, theta_velocity, pulse_per_sec
setPosition(theta, phi)               # direct position set with bounds clamping
setPulsesPerRevolution(pulses)
setMaxVelocity(v) / setMaxAcceleration(a)
setThetaBounds(min, max) / setPhiBounds(min, max)
```

**CHOP output** (`const_robot` Constant CHOP):
```
const0value = theta
const1value = phi
const2value = pulse_per_sec + 100000  # offset for serial communication
```

**Velocity/acceleration clamping in `SetState`:**
- Computes velocity from positional delta / dt
- Clamps velocity to `max_theta_velocity` / `max_phi_velocity`
- Computes acceleration from velocity delta / dt
- Clamps acceleration, back-calculates velocity if exceeded
- Uses `_shortestAngleDelta()` for theta to handle 360° wraparound

---

### `RobotControllerEXT` class (one instance in RobotController COMP)
**Role:** Owns all motion logic — behaviors, blending, fading, and applying states to all robots.

**Initialization:**
- Auto-discovers robots by looking for `Robot1`–`Robot6` as siblings: `ownerComp.parent().op(f'Robot{i}')`
- Uses `time.time()` for real clock delta time (not TD timeline)

**Core data structures:**

**Behaviors dict** — each behavior has a weight and parameters:
```python
behaviors = {
    'sine':  { 'weight': 1.0, 'theta_velocity': 30.0, 'frequency_phi': 0.5,
               'phase_shift': 0.0, 'phase_theta': 0.0,
               'bias_phi': 90.0, 'amplitude_phi': 30.0 },
    'wave':  { 'weight': 0.0, 'frequency_theta': 1.0, 'frequency_phi': 1.0,
               'phase_shift': 0.0, 'bias_theta': 180.0, 'bias_phi': 90.0,
               'amplitude_theta': 45.0, 'amplitude_phi': 30.0 },
    'noise': { 'weight': 0.0, 'frequency_theta': 3.0, 'frequency_phi': 3.0,
               'phase_shift': 0.0, 'bias_theta': 180.0, 'bias_phi': 90.0,
               'amplitude_theta': 10.0, 'amplitude_phi': 10.0 }
}
```

**State dict format** (per robot, passed between all methods):
```python
{ 'theta': float, 'phi': float, 'led': [r, g, b] }
```

**Update loop** (`Update()` — called every frame):
```
1. Compute real dt via time.time()
2. If paused → re-apply last_states and return
3. _updateFade(dt) — advance fade if active
4. _smoothBehaviorParameters(dt) — exponential smooth all params toward targets
5. For each behavior with weight > 0: run behavior, blend states by weight
6. If fading: override final_states with interpolated faded_states
7. _applyStates(final_states, dt) → calls robot.SetState() + robot.PushToCHOP()
```

**Behaviors:**
- `sine` — theta rotates at constant velocity, phi oscillates sinusoidally per robot with phase offset
- `wave` — both theta and phi oscillate sinusoidally using absolute time
- `noise` — same as wave with different parameters (higher frequency, smaller amplitude)

**Blending:**
- Multiple behaviors can run simultaneously with weights
- States are blended: `blended = lerp(stateA, stateB, weightB)`
- `_blendStates()` interpolates theta (shortest arc), phi, and led linearly

**Parameter smoothing:**
- `smoothed_behaviors` mirrors `behaviors` but values lag behind via exponential smoothing
- `alpha = 1 - exp(-smoothing_speed * dt)` (default speed: 6.0)
- Prevents parameter snapping when values are changed externally

**Fade system:**
- `fadeTo(target, duration)` — smoothly transitions from current states to a target behavior
- Uses `_easeInOutCubic(t)` for smooth interpolation
- On completion: zeros all weights, sets target weight to 1.0

**Direct controls (public API):**
```python
Update()                              # call every frame
fadeTo(target, duration=2.0)          # fade to 'sine', 'wave', or 'noise'
setWeight(name, weight)               # set behavior weight directly
setFrequency(name, freq_theta, freq_phi)
setThetaVelocity(velocity)            # sets sine theta_velocity
setPhaseShift(name, phase_shift)
setBias(name, bias_theta, bias_phi)
zeroAll()                             # set all weights to 0
stop()                                # pause motion
resume()                              # resume motion
servo_min()                           # all robots to phi=-135
servo_max()                           # all robots to phi=0
GetAllStates()                        # returns list of GetState() dicts
```

---

## Serial Protocol
- `pulse_per_sec + 100000` offset is added before sending to Arduino
- Full protocol design TBD — Arduino firmware not yet written
- Arduino receives per-robot: pulse_per_sec (theta), phi servo angle, LED RGB
- Arduino outputs: homing sensor state per robot

---

## Files
- `RobotEXT.py` — Robot state container and CHOP output
- `RobotControllerEXT.py` — All motion logic, behaviors, blending, fading
- `robot_project_context.md` — This file
