# Robot Arm Installation — Project Context

## Overview
A kinetic art installation consisting of **6 robot arms** mounted in a physical space, controlled in real time via TouchDesigner and an Arduino. Each arm moves in spherical coordinates, has a ribbon attached to its end effector, and is equipped with a DMX color fixture. The system supports synchronized choreography, individual control, and a library of motion behaviors.

---

## Hardware

### Robot Arm Mechanics
- Each arm moves in **spherical coordinates** defined by two motors:
  - **Theta motor** — stepper motor, azimuthal rotation in the **XZ plane** (around Y axis)
    - Range: 0–360° (continuous, wraps)
    - Controlled by **velocity** — converted to **pulses per second** for the stepper driver
    - Formula: `pulse_per_sec = theta_velocity * pulses_per_revolution`
    - Velocity unit: **rotations per second (rps)**
  - **Phi motor** — 270° servo motor, elevation from Y+ axis
    - Range: **-135° to +135°** (0° = pointing straight up along Y+)
    - Controlled by **angle** (direct servo position)
- A **ribbon** is attached to the end of each arm — can get caught near phi ≈ 0 at low speed

### Coordinate System
```
Y+ = phi 0°  (arm pointing up)
XZ plane = phi 90°  (arm horizontal)
Y- = phi 180° (unreachable, servo limit is ±135°)

Theta: azimuth in XZ plane, 0–360°, wraps continuously
Phi:   elevation from Y+, -135° to +135°

xyz mapping:
  y = cos(phi)
  x = sin(phi) * sin(theta)
  z = sin(phi) * cos(theta)
  theta = atan2(x, z)
```

### Stepper Motor (Theta)
- Driver: **DM860** with built-in optocouplers (PUL, DIR, ENA)
- **800 pulses per revolution** (must match DM860 DIP switch setting)
- Output: `pulse_per_sec + 100000` (offset for serial protocol; 100000 = stopped)
- Grounding: Arduino ground must be isolated from 60V system to protect optocouplers

### Servo Motor (Phi)
- 270° servo, powered at **25.2V**
- Control signal: PWM from Arduino
- Output: phi angle in [-135, 135]

### Homing Sensor
- Powered at **25.2V**
- Signal voltage-divided to ~4.13V (5.6kΩ / 1.1kΩ divider)

### DMX Color Fixture
- One RGB fixture per robot (3 channels: R, G, B)
- Output via TouchDesigner Art-Net/DMX
- Represented as `led: [r, g, b]` in state dicts

---

## Electronics & Power

### Power System (per robot)
- **60V supply** — stepper driver (DM860)
- **25.2V DC-DC buck** (non-isolated from 60V) — servo + homing sensor
- Arduino powered from **isolated 5V supply** — must NOT share ground with 60V system
- USB programming via **USB isolator**

### Known Hardware Issue
Arduino 3.3V regulator was destroyed when grounds were shared. Fix: fully isolated 5V supply for Arduino, USB isolator during programming.

---

## Software — TouchDesigner

### Node Structure
```
/project1
    /RobotController    Base COMP — RobotControllerEXT extension
    /Robot1             Base COMP — RobotEXT extension
        const_robot     Constant CHOP — receives motor output
    /Robot2 ... /Robot6 (identical structure)
```

### Execute DAT (frame loop)
`Update()` is called every frame from an Execute DAT:
```python
op('RobotController').ext.RobotControllerEXT.Update()
```
The controller reads its own wall-clock time internally. Do NOT pass dt externally.

### TouchDesigner Extension Promotion
Methods on RobotEXT extensions are promoted to the operator level, so `r.SetState(...)` works. However, **attribute assignment** (`r.theta_velocity = x`) does NOT go through the extension — always use methods (e.g. `r.Halt()`).

---

## Python Extensions

---

### `RobotEXT` — `DATS/RobotEXT.py`
State container for a single robot arm. One instance per Robot COMP.

#### State
| Attribute | Unit | Default | Notes |
|---|---|---|---|
| `theta` | deg | 0.0 | [0, 360) source of truth |
| `phi` | deg | 90.0 | [-135, 135] |
| `theta_velocity` | rps | 0.0 | computed each frame |
| `phi_velocity` | deg/s | 0.0 | computed each frame |

#### Limits (configurable)
| Attribute | Default | Notes |
|---|---|---|
| `pulses_per_revolution` | 800 | must match DM860 DIP switch |
| `max_theta_velocity` | 1.5 rps | |
| `max_phi_velocity` | 540.0 deg/s | equivalent to 1.5 rps |
| `max_theta_acceleration` | 2.3 rps/s | 0 = unlimited |
| `max_phi_acceleration` | 1.0 deg/s² | 0 = unlimited |
| `min_phi` / `max_phi` | -135 / 135 | servo hardware limits |

#### Key Methods
```python
SetState(theta, phi, ledMatrix, dt=None)
    # Sets position. If dt > 0: computes velocity from delta/dt,
    # clamps by max_velocity and max_acceleration.
    # If dt is None or 0: snaps position, zeroes velocity.

PushToCHOP()
    # Writes to const_robot Constant CHOP:
    #   const0value = theta
    #   const1value = phi
    #   const2value = (-theta_velocity * pulses_per_revolution) + 100000

Halt()
    # Zeros velocity and calls PushToCHOP(). Use this from outside —
    # never set r.theta_velocity directly (TD wrapper issue).

GetState() → dict
    # Returns: theta, phi, theta_velocity, pulse_per_sec

setMaxThetaVelocity(v)       # rps
setMaxPhiVelocity(v)         # deg/s
setMaxThetaAcceleration(a)   # rps/s
setMaxPhiAcceleration(a)     # deg/s²
setPhiBounds(min, max)
setPulsesPerRevolution(p)
```

#### CHOP Output Format
```
const0value = theta (deg)
const1value = phi (deg)
const2value = pulse_per_sec + 100000
    where pulse_per_sec = -theta_velocity * pulses_per_revolution
    100000 = stopped (zero velocity)
    < 100000 = negative direction
    > 100000 = positive direction
```

---

### `RobotControllerEXT` — `DATS/RobotControllerEXT.py`
Orchestrates all 6 robots. Manages behaviors, blending, fading, stop/resume ramps, homing, and global limits.

#### Initialization
```python
self.robots = [ownerComp.parent().op(f'Robot{i}') for i in range(1, 7)]
```
Auto-discovers Robot1–Robot6 as siblings of the ownerComp.

---

### Update Loop

```
Update() called every frame:
  1. Skip if dt < 0.001 (double-cook guard — TD fires Execute DATs multiple times/frame)
  2. If quaternion weight > 0 and CHOP configured: read rotation axis from CHOP
  3. Homing state transitions
  4. If paused: return
  5. Compute effective_dt (scaled by stop/resume ramp)
  6. Accumulate self.time += effective_dt
  7. Smooth behavior parameters
  8. Blend all active behaviors
  9. Apply states to robots via SetState + PushToCHOP
```

**Double-cook guard:** TD Execute DATs fire multiple times per frame with dt≈0. The guard `if dt < 0.001: return` prevents the second cook from zeroing velocities.

---

### Behavior System

All behaviors live in `BEHAVIOR_DEFAULTS` and `_behavior_fns`. Each behavior:
- Has a **weight** in `self.weights` (independent of parameters)
- Has **target parameters** in `self.behaviors[name]`
- Has **smoothed parameters** in `self.smoothed_params[name]` (exponential lag toward target)
- Returns a `list[{'theta': float, 'phi': float, 'led': [r,g,b]}]` for all 6 robots

Multiple behaviors can run simultaneously — states are **normalized weighted blended**.

#### Parameter Smoothing
```python
alpha = 1 - exp(-smoothing_speed * dt)   # default speed = 6.0
smoothed = smoothed + (target - smoothed) * alpha
```
Prevents snapping when parameters are changed externally. Non-numeric params (e.g. `rotation_axis` list) pass through unchanged.

---

### Behavior Reference

#### `sine`
Theta rotates at constant velocity; phi oscillates sinusoidally per robot.
Uses per-robot **phase accumulators** (`sine_theta_positions`, `phase_phi`) — frequency changes are seamless.
```python
Parameters:
  theta_velocity  # rps — constant spin speed (default 0.1)
  frequency_phi   # Hz (default 0.5)
  phase_shift     # rad offset between robots (phi)
  phase_theta     # deg offset between robots (theta)
  bias_phi        # deg — phi center position (default -90)
  amplitude_phi   # deg — phi oscillation amplitude (default 30)
```

#### `wave`
Both theta and phi oscillate sinusoidally using `self.time` (absolute).
```python
Parameters:
  frequency_theta, frequency_phi  # Hz
  phase_shift                     # rad between robots
  bias_theta, bias_phi            # deg — center positions
  amplitude_theta, amplitude_phi  # deg — oscillation amplitudes
```

#### `noise`
Same as `wave` with higher frequency, smaller amplitude defaults. Good for subtle perturbation.

#### `circular`
Traces a circle in a tilted vertical plane using per-robot angle accumulators.
```python
Parameters:
  angular_speed   # deg/s
  tilt_angle      # deg from Y axis (min 45)
  azimuth_offset  # deg in XZ plane
  phase_shift     # rad between robots
```

#### `quaternion`
Rotates each robot's arm vector around a 3D axis (Rodrigues' formula). Arms move in the plane perpendicular to the axis. Per-robot unit vectors stored in `quaternion_positions` (lazily initialized on first use).
```python
Parameters:
  rotation_axis  # [x, y, z] — 3D vector to rotate around
  angular_speed  # rps
  phase_shift    # rad between robots (default 1.047 = 2π/6)
```
**Live CHOP input:** `setRotationAxisCHOP('Rotation_Vector')` — reads x,y,z channels every frame when quaternion weight > 0.
**Re-initialize positions:** set `ext.quaternion_positions = None` or call `setQuaternionPhaseShift()`.

#### `circle_zy`
Traces a circle on the unit sphere in the plane `x = x_offset`, centered at `(x_offset, 0, 0)`. Radius = `sqrt(1 - x_offset²)`. Uses per-robot angle accumulators.
```python
Parameters:
  x_offset      # X plane position (-1 < x < 1); default 0.5
                # x=0 → full great circle in ZY plane
                # x→±1 → smaller circle near pole
  angular_speed # rps
  phase_shift   # rad between robots
```

#### `figure8`
Lissajous figure-8: theta at frequency f, phi at 2f. Per-robot phase accumulation for smooth frequency transitions.
```python
Parameters:
  frequency          # Hz — base (phi runs at 2x)
  amplitude_theta    # deg — half-width
  amplitude_phi      # deg — half-height
  bias_theta         # deg — center azimuth
  bias_phi           # deg — center elevation
  phase_shift        # rad between robots
  theta_phase_shift  # rad — additional phase offset on theta only
```

#### `figure9`
Inverse Lissajous: theta at 2f, phi at f. Same parameters as figure8.
```python
Parameters:
  frequency          # Hz — base (theta runs at 2x)
  amplitude_theta, amplitude_phi
  bias_theta, bias_phi
  phase_shift
  theta_phase_shift  # rad — offset on the doubled-frequency theta term
```

#### `flip`
Binary phi toggle — all robots snap to phi=-135 (state=0) or phi=135 (state=1).
```python
Parameters:
  state       # 0.0 or 1.0
  bias_theta  # deg — fixed azimuth while flipped
```

#### `stepper_speed_control`
Theta (stepper) rotates continuously at `stepper_speed` rps; robot i's theta is offset by
`i * stepper_phase_offset` degrees. Phi is set directly from a time-delayed `phi_position`:
robot i reads the value from `i * phi_delay` seconds ago, creating a cascading wave on the phi axis.
```python
Parameters:
  stepper_speed        # rps — theta rotation speed (default 0.1)
  stepper_phase_offset # deg — per-robot theta offset: robot i += i * offset (default 0)
  phi_position         # deg — phi setpoint for robot 0, range -135 to 135 (default 0)
  phi_delay            # s — cascading delay: robot i reads phi_position from i * phi_delay seconds ago (default 0)
```

#### `stepper_direct_control`
Direct position control for both theta and phi with independent cascading delays. Positions are
set directly each frame; velocity is derived by `RobotEXT.SetState` subject to hardware limits.
```python
Parameters:
  stepper_position # deg — theta setpoint for robot 0 (default 0)
  stepper_delay    # s — cascading delay: robot i reads stepper_position from i * stepper_delay seconds ago (default 0)
  phi_position     # deg — phi setpoint for robot 0, range -135 to 135 (default 0)
  phi_delay        # s — cascading delay: robot i reads phi_position from i * phi_delay seconds ago (default 0)
```
**Cascading:** both axes delay independently. E.g. `stepper_delay=0, phi_delay=0.5` snaps all
robots to the same azimuth while phi ripples through them as a wave.

---

### Stop / Resume / Playback Controls

All ramps use `decel_duration` (default 0.5s, shared by stop and resume).

```python
hardStop()
    # Instant: zeroes velocity, writes 100000 to all CHOPs immediately.
    # Cancels homing, fades, ramps. Sets paused=True.

stop()
    # Smooth decel: scales effective_dt from 1→0 over decel_duration.
    # Velocity reaches zero naturally. Then Halt() fires and paused=True.
    # Safe to call repeatedly — resets ramp each time.

resume()
    # Smooth accel: scales effective_dt from 0→1 over decel_duration.
    # Clears stopping/paused flags, sets resuming=True.

setDecelDuration(seconds)
    # Controls both stop and resume ramp duration (min 0.05s).

servo_min()    # All robots → phi=-135, theta=0, paused=True
servo_max()    # All robots → phi=135,  theta=0, paused=True
servo_zero()   # All robots → phi=0,    theta=0, paused=True
```

---

### Fade System

Smooth transitions between behaviors using cubic ease-in-out interpolation.

```python
fadeTo(target_behavior, duration=2.0)
    # Interpolates from current states to predicted end states of target.
    # On completion: zeros all weights, sets target weight to 1.0.
    # Supported targets: 'sine', 'wave', 'noise', 'circular', 'quaternion',
    #                    'circle_zy', 'figure8', 'figure9', 'flip'
```

For `sine` and `circular`, the fade endpoint is **predicted** `duration` seconds ahead (seamless landing). Other behaviors sample their current state as the endpoint.

---

### Homing Sequence

State machine: `None → 'decelling' → 'waiting' → 'resuming' → None`

```python
startHoming(
    trigger_op  = None,     # op name to pulse when stopped (homing button)
    trigger_par = 'Pulse',  # parameter to pulse on that op
    serial_dat  = None,     # Serial DAT op name to poll for done string
    done_string = 'HOMED'   # text that signals homing complete
)
```

**Sequence:**
1. `stop()` — decelerates over `decel_duration`
2. Saves current phi values for all robots
3. Transitions to `'waiting'`, pulses homing trigger op
4. Polls `serial_dat` every frame for `done_string`
5. On match: snaps all robots to **theta=315**, restores saved phi values
6. `resume()` — accelerates back up over `decel_duration`

**Manual completion** (when not using serial polling):
```python
ext.onHomingComplete()   # call from a DAT Execute or CHOP Execute
```

**Op search:** looks in `ownerComp` first, then `ownerComp.parent()`.

---

### Global Acceleration Limits

Applied per-robot via method calls (avoids TD wrapper issue):

```python
setMaxThetaAcceleration(accel)   # rps/s — applied to all robots
setMaxPhiAcceleration(accel)     # deg/s² — applied to all robots
# Pass 0 for unlimited
```

These write to each `RobotEXT` instance via `r.setMaxThetaAcceleration()`.

---

### Full Public API

#### Behavior control
```python
setWeight(name, weight)                          # direct weight set
fadeTo(target, duration=2.0)                     # smooth transition
zeroAll()                                        # set all weights to 0
setParam(behavior, key, value)                   # generic param setter
```

#### Sine
```python
setThetaVelocity(velocity)                       # rps
setPhaseShift('sine', phase_shift)               # rad
setBias('sine', bias_theta=None, bias_phi=None)  # deg
setFrequency('sine', frequency_phi=0.5)
```

#### Wave / Noise
```python
setFrequency('wave', frequency_theta, frequency_phi)
setPhaseShift('wave', phase_shift)
setBias('wave', bias_theta, bias_phi)
```

#### Circular
```python
setCircularSpeed(deg_per_sec)
setCircularTilt(tilt_angle_deg)     # min 45
setCircularAzimuth(azimuth_deg)
```

#### Quaternion
```python
setRotationAxis(x, y, z)
setRotationAxisCHOP(op_name)        # live CHOP feed (x,y,z channels); None to disconnect
setQuaternionSpeed(rps)
setQuaternionPhaseShift(rad)        # also resets quaternion_positions
```

#### Circle ZY
```python
setCircleZYOffset(x_offset)         # -1 < x < 1
setCircleZYSpeed(rps)
```

#### Figure8 / Figure9
```python
setParam('figure8', 'frequency', 0.25)
setParam('figure8', 'amplitude_theta', 60.0)
setParam('figure8', 'amplitude_phi', 30.0)
setParam('figure8', 'bias_theta', 180.0)
setParam('figure8', 'bias_phi', 0.0)
setParam('figure8', 'phase_shift', 1.047)
setParam('figure8', 'theta_phase_shift', 0.0)
# Same keys for 'figure9'
```

#### Flip
```python
setFlipState(0)   # phi = -135 for all robots
setFlipState(1)   # phi =  135 for all robots
```

#### StepperSpeedControl
```python
setStepperSpeed(rps)                  # theta rotation speed
setStepperPhaseOffset(deg)            # per-robot theta offset (robot i += i * offset)
setStepperSpeedPhi(phi_position)      # phi setpoint (-135 to 135)
setStepperSpeedPhiDelay(seconds)      # cascading phi delay between robots
```

#### StepperDirectControl
```python
setStepperDirectPosition(deg)         # theta setpoint (0–360)
setStepperDirectDelay(seconds)        # cascading theta delay between robots
setStepperDirectPhi(deg)              # phi setpoint (-135 to 135)
setStepperDirectPhiDelay(seconds)     # cascading phi delay between robots
```

#### Playback
```python
hardStop()
stop()
resume()
setDecelDuration(seconds)
setMaxThetaAcceleration(rps_per_s)
setMaxPhiAcceleration(deg_per_s2)
servo_min() / servo_max() / servo_zero()
```

#### Homing
```python
startHoming(trigger_op, trigger_par, serial_dat, done_string)
onHomingComplete()
```

#### Output
```python
GetAllStates()   # returns list of {theta, phi, theta_velocity, pulse_per_sec}
```

---

## Serial Protocol

- **Baud rate / format:** defined in Arduino firmware
- **Value sent:** `pulse_per_sec + 100000` where `pulse_per_sec = -theta_velocity * 800`
  - 100000 = stopped
  - 100000 + N = forward at N pulses/sec
  - 100000 - N = reverse at N pulses/sec
- **Homing done string:** Arduino sends a text string (default `'HOMED'`) when homing is complete
- Arduino receives per-robot: pulse_per_sec (theta velocity), phi servo angle, LED RGB
- Arduino outputs: homing sensor state per robot

---

## Files

| File | Role |
|---|---|
| `DATS/RobotEXT.py` | Per-robot state container and CHOP output |
| `DATS/RobotControllerEXT.py` | All motion logic: behaviors, blending, fading, stop/resume, homing |
| `DATS/MotionRecorderEXT.py` | Records robot motion into named clips; registers clips as behaviors |
| `DATS/ShowSequencerEXT.py` | Time-coded show: keyframe tracks + event cues driving the controller |
| `QuaternionRotationScriptCallback/QuaternionScriptCallback.py` | Reference Script CHOP for single-arm quaternion rotation (not the main controller) |
| `robot_project_context.md` | This file |

---

---

## MotionRecorderEXT — `DATS/MotionRecorderEXT.py`

Records per-robot theta/phi motion into named clips. Each clip is registered as a dynamic
behavior on the controller so it can be weighted, blended, and faded identically to built-ins.

**COMP:** `recorder_comp` (Base COMP, sibling of `controller_comp`)
**Update order:** call after `controller_comp` Update so `last_states` is fresh.

### Recording
```python
rec = op('recorder_comp').ext.MotionRecorderEXT

rec.startRecording('take_1')            # begin capture at 30 fps
rec.startRecording('take_1', sample_rate=60, include_led=True)  # options
rec.stopRecording()                     # finalizes; registers clip on controller

rec.isRecording                         # bool property
rec.recordingTime                       # elapsed seconds since startRecording()
```

### Clip playback controls
Once registered, a clip behaves exactly like any behavior on the controller.
These methods control the clip's internal playback cursor:
```python
rec.resetClip('take_1')                 # rewind to t=0
rec.seekClip('take_1', 12.5)            # jump to 12.5s
rec.setClipSpeed('take_1', 0.5)         # half speed; -1.0 = reverse
rec.setClipLoop('take_1', False)        # one-shot (holds last frame)
rec.getClipState('take_1')              # → {'t': ..., 'speed': ..., 'loop': ...}
```

### Blending clips (via controller)
```python
ctrl = op('controller_comp').ext.RobotControllerEXT
ctrl.setWeight('take_1', 1.0)           # switch to clip
ctrl.fadeTo('take_1', 2.0)             # smooth fade in
ctrl.setWeight('take_1', 0.5)          # blend 50/50 with whatever else is running
```

### Clip management
```python
rec.listClips()                         # ['take_1', 'take_2', ...]
rec.getClipInfo('take_1')              # {'name', 'duration', 'frames', 'sample_rate'}
rec.deleteClip('take_1')               # removes from controller too
```

### Persistence
```python
rec.saveClips('C:/show/clips.json')
rec.loadClips('C:/show/clips.json')     # re-registers all clips on controller
```

---

## ShowSequencerEXT — `DATS/ShowSequencerEXT.py`

Table-driven show sequencer. Reads a **Table DAT** (one row per segment) and drives
`controller_comp` by interpolating behavior parameters over time.

**COMP:** `sequencer_comp` (Base COMP, sibling of `controller_comp`)
**Update order:** call BEFORE `controller_comp` Update so parameters land this frame.

### Table DAT format

Create a Table DAT named `sequencer_table` (or configurable). First row must be a header row
with exactly these column names:

| `time_start` | `duration` | `behavior` | `blend_start` | `blend_end` | `params_start` | `params_end` |
|---|---|---|---|---|---|---|
| 0.0 | 10.0 | sine | | | theta_velocity=0.1 bias_phi=-90 | theta_velocity=0.3 bias_phi=-45 |
| 10.0 | 4.0 | sine/wave | 0.0 | 1.0 | theta_velocity=0.2 | |
| 14.0 | 8.0 | figure8 | | | frequency=0.25 amplitude_theta=60 | frequency=0.5 amplitude_theta=30 |

**Column definitions:**
- `time_start` — seconds; when this segment becomes active
- `duration` — seconds; how long params interpolate from start → end; final state holds after
- `behavior` — single name (`sine`) or cross-fade (`sine/wave`); for a cross-fade `blend_start`/`blend_end` control the mix
- `blend_start` / `blend_end` — 0–1 weight of the second behavior; 0 = all first, 1 = all second; only used when `behavior` contains `/`
- `params_start` / `params_end` — space-separated `key=value` pairs for behavior_a parameters; empty cell = no change from defaults

**Rules:**
- The sequencer zeros all behavior weights on every frame during playback — it fully owns weight state
- Params apply only to behavior_a; configure behavior_b defaults before the show starts
- After a segment's duration ends its final state holds until the next segment's `time_start`
- Segments are sorted by `time_start` automatically on load; order in the table doesn't matter

### Playback
```python
seq = op('sequencer_comp').ext.ShowSequencerEXT

seq.play()           # start / resume from current position
seq.pause()          # freeze playhead; controller holds current state
seq.stop()           # pause + rewind to t=0
seq.seek(30.0)       # jump to 30s; immediately applies that segment's state
seq.setLoop(True)

seq.seq_time         # current playhead position (seconds, read-only)
seq.playing          # bool
seq.duration         # auto-computed from last segment end after loadFromDAT()
```

### Loading the table
```python
# Default: reads op named 'sequencer_table' in parent network
seq.loadFromDAT()

# Specify a different op name
seq.loadFromDAT('my_show_table')
seq.setDATName('my_show_table')   # change default for future reloads
seq.reloadDAT()                   # re-parse without changing playback state

seq.listSegments()                # prints all parsed segments to console
```

### Params format reference
The `params_start` and `params_end` columns accept any parameter key valid for the named behavior:
```
# sine
theta_velocity=0.1 frequency_phi=0.5 phase_shift=0 bias_phi=-90 amplitude_phi=30

# wave / noise
frequency_theta=1.0 frequency_phi=1.0 phase_shift=0 bias_theta=180 bias_phi=90 amplitude_theta=45 amplitude_phi=30

# circular
angular_speed=30 tilt_angle=45 azimuth_offset=0 phase_shift=0

# figure8 / figure9
frequency=0.25 amplitude_theta=60 amplitude_phi=30 bias_theta=180 bias_phi=0 phase_shift=1.047 theta_phase_shift=0

# stepper_speed_control
stepper_speed=0.1 stepper_phase_offset=0 phi_position=0 phi_delay=0

# stepper_direct_control
stepper_position=0 stepper_delay=0 phi_position=0 phi_delay=0

# flip
state=0 bias_theta=0
```

---

## Node structure (updated)

```
/project1
    /controller_comp    Base COMP — RobotControllerEXT
    /sequencer_comp     Base COMP — ShowSequencerEXT
    /recorder_comp      Base COMP — MotionRecorderEXT
    /Robot1 ... /Robot6
```

Execute DAT (call in this order every frame):
```python
op('sequencer_comp').ext.ShowSequencerEXT.Update()
op('controller_comp').ext.RobotControllerEXT.Update()
op('recorder_comp').ext.MotionRecorderEXT.Update()
```

---

## Common Patterns

### Switch behavior instantly
```python
ctrl = op('controller_comp').ext.RobotControllerEXT
ctrl.setWeight('figure8', 1.0)
ctrl.setWeight('sine', 0.0)
```

### Fade to a behavior smoothly
```python
ctrl.fadeTo('quaternion', 2.0)
```

### Emergency stop
```python
ctrl = op('controller_comp').ext.RobotControllerEXT
ctrl.hardStop()
```

### Graceful stop and restart
```python
ctrl.stop()        # decelerates
# ... later ...
ctrl.resume()      # accelerates back up
```

### Full homing sequence
```python
ctrl.startHoming(
    trigger_op  = 'button_home',
    trigger_par = 'Pulse',
    serial_dat  = 'serial1',
    done_string = 'HOMED'
)
```

### Live CHOP-driven quaternion axis
```python
ctrl.setRotationAxisCHOP('Rotation_Vector')  # set once; reads every frame
ctrl.fadeTo('quaternion', 2.0)
```

### Record a motion clip and play it back
```python
rec  = op('recorder_comp').ext.MotionRecorderEXT
ctrl = op('controller_comp').ext.RobotControllerEXT

rec.startRecording('take_1')
# ... let robots move for some time ...
rec.stopRecording()

ctrl.fadeTo('take_1', 2.0)             # blend into the recording
```

### Run a table-driven show
```
# In a Table DAT named 'sequencer_table':
# time_start | duration | behavior  | blend_start | blend_end | params_start                      | params_end
# 0.0        | 8.0      | sine      |             |           | theta_velocity=0.1 bias_phi=-90   | theta_velocity=0.2 bias_phi=-60
# 8.0        | 4.0      | sine/wave | 0.0         | 1.0       | theta_velocity=0.2                |
# 12.0       | 10.0     | wave      |             |           | frequency_phi=0.5 amplitude_phi=30| frequency_phi=1.5 amplitude_phi=60
```
```python
seq = op('sequencer_comp').ext.ShowSequencerEXT
seq.loadFromDAT('sequencer_table')
seq.play()
```

### Adding a new behavior
1. Add entry to `BEHAVIOR_DEFAULTS` with default parameters
2. Add per-robot accumulator list to `__init__` if stateful
3. Implement `_mybehavior(self, dt)` returning `list[{'theta', 'phi', 'led'}]`
4. Register: `'mybehavior': self._mybehavior` in `_behavior_fns`
5. Add setter methods as needed
