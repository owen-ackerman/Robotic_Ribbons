import math
import time
from collections import deque


class RobotControllerEXT:
    """
    Owns all motion logic for 6 robot arms — behaviors, blending, fading, and
    applying states to individual RobotEXT instances each frame.

    Adding a new behavior (e.g. 'quaternion') requires exactly three steps:
      1. Add its default parameters to BEHAVIOR_DEFAULTS below.
      2. Implement a _<name>(self, dt) method that returns a states list.
      3. Register it in self._behavior_fns inside __init__.
    """

    # Behavior default parameters
    # Each key maps to the parameters used by that behavior's compute method.
    # Weights live separately in self.weights — they are NOT stored here.

    BEHAVIOR_DEFAULTS = {
        'sine': {
            'theta_velocity': 0.1,    # rps constant rotation speed
            'frequency_phi':   0.5,   # Hz
            'phase_shift':     0.0,   # rad offset between robots (phi)
            'phase_theta':     0.0,   # deg offset between robots (theta)
            'bias_phi':       -90.0,  # deg — phi rest position
            'amplitude_phi':  30.0,   # deg — phi oscillation amplitude
        },
        'wave': {
            'frequency_theta': 1.0,
            'frequency_phi':   1.0,
            'phase_shift':     0.0,
            'bias_theta':    180.0,
            'bias_phi':       90.0,
            'amplitude_theta': 45.0,
            'amplitude_phi':   30.0,
        },
        'noise': {
            'frequency_theta': 3.0,
            'frequency_phi':   3.0,
            'phase_shift':     0.0,
            'bias_theta':    180.0,
            'bias_phi':       90.0,
            'amplitude_theta': 10.0,
            'amplitude_phi':   10.0,
        },
        'circular': {
            'angular_speed':  30.0,   # deg/s angular rotation speed
            'tilt_angle':     45.0,   # deg — center elevation
            'azimuth_offset':  0.0,   # deg in XZ plane (0-360)
            'phase_shift':     0.0,   # rad offset between robots
            'amplitude':      45.0,   # deg — angular radius of the circular path
        },
        'quaternion': {
            'rotation_axis': [0.0, 1.0, 0.0],  # 3D vector; arm rotates in plane perp. to this
            'angular_speed': 0.1,               # rps
            'phase_shift':   1.047,             # rad offset between robots (2π/6 for 6 robots)
        },
        'circle_zy': {
            'x_offset':     0.72,    # X position of the plane (-1 < x < 1); radius = sqrt(1 - x²)
            'angular_speed': 0.84,   # rps
            'phase_shift':   0.0,   # rad offset between robots
        },
        'flip': {
            'state':       0.0,    # 0 → phi = -135,  1 → phi = 135
            'bias_theta':  0.0,    # deg — fixed azimuth while flipped
        },
        'figure8': {
            'frequency':         0.25,  # Hz — base frequency (phi runs at 2x this)
            'amplitude_theta':  60.0,   # deg — half-width
            'amplitude_phi':    30.0,   # deg — half-height
            'bias_theta':      180.0,   # deg — center azimuth
            'bias_phi':          0.0,   # deg — center elevation
            'phase_shift':       1.047, # rad — offset between robots
            'theta_phase_shift': 0.0,   # rad — phase offset applied to theta only
        },
        'figure9': {
            'frequency':         0.25,  # Hz — base frequency (theta runs at 2x this)
            'amplitude_theta':  40.0,   # deg — half-width
            'amplitude_phi':    135.0,  # deg — half-height
            'bias_theta':       90.0,   # deg — center azimuth
            'bias_phi':          0.0,   # deg — center elevation
            'phase_shift':       1.047, # rad — offset between robots
            'theta_phase_shift': 0.0,   # rad — phase offset applied to theta only
        },
        'stepper_speed_control': {
            'stepper_speed':        0.7,  # rps — theta rotation speed
            'stepper_phase_offset': 0.0,  # deg — per-robot theta offset (robot i += i * offset)
            'phi_position':         60.0, # deg — phi setpoint for robot 0 (-135 to 135)
            'phi_delay':            0.0,  # s — cascading delay: robot i reads phi_position from i * phi_delay seconds ago
        },
        'stepper_direct_control': {
            'stepper_position': 0.0,  # deg — theta setpoint for robot 0
            'stepper_delay':    0.0,  # s — cascading delay: robot i reads stepper_position from i * stepper_delay seconds ago
            'phi_position':     90.0, # deg — phi setpoint for robot 0 (-135 to 135)
            'phi_delay':        0.0,  # s — cascading delay: robot i reads phi_position from i * phi_delay seconds ago
        },
        'home_sweep': {
            'theta_start': 135.0,  # deg — starting theta (post-homing position)
            'theta_speed':   0.3,  # rps — rate of decrease toward 0
            'phi_target':    0.0,  # deg — phi destination
            'phi_speed':    45.0,  # deg/s — rate of phi interpolation toward phi_target
        },
    }

    def __init__(self, ownerComp):
        self.ownerComp = ownerComp

        self.robots = [ownerComp.parent().op(f'Robot{i}') for i in range(1, 7)]
        self.robots = [r for r in self.robots if r]

        self.time          = 0.0
        self.previous_time = time.time()

        # Behavior weights — independent from behavior parameters
        self.weights = {name: 0.0 for name in self.BEHAVIOR_DEFAULTS}
        self.weights['sine'] = 0.0

        # Per-behavior robot activation masks — 1.0 = active, 0.0 = inactive
        self.robot_masks = {name: [1.0] * 6 for name in self.BEHAVIOR_DEFAULTS}

        # Target parameter values (what the user sets)
        self.behaviors = {name: dict(defaults) for name, defaults in self.BEHAVIOR_DEFAULTS.items()}
        # Smoothed parameter values (lag toward targets — what behaviors actually read)
        self.smoothed_params = {name: dict(defaults) for name, defaults in self.BEHAVIOR_DEFAULTS.items()}
        self.parameter_smoothing_speed = 6.0

        # Per-robot phase accumulators for stateful behaviors
        self.sine_theta_positions  = [0.0] * len(self.robots)
        self.phase_phi             = [0.0] * len(self.robots)
        self.theta_directions      = [1]   * len(self.robots)
        self.circular_angles       = [0.0] * len(self.robots)
        self.circle_zy_angles      = [0.0] * len(self.robots)
        self.figure8_phases        = [0.0] * len(self.robots)
        self.figure9_phases        = [0.0] * len(self.robots)
        # Per-robot unit vectors tracking current arm direction for quaternion behavior
        self.quaternion_positions  = None  # lazily initialized on first use
        # home_sweep per-robot accumulators — None until first use; reset via resetHomeSweep()
        self.home_sweep_theta = None
        self.home_sweep_phi   = None

        # StepperSpeedControl — per-robot theta accumulators
        self.stepper_speed_theta = [0.0] * len(self.robots)
        # Rolling time-stamped histories for cascading delay (up to ~60 s at 60 fps)
        self._stepper_speed_phi_history    = deque(maxlen=3600)
        self._stepper_direct_theta_history = deque(maxlen=3600)
        self._stepper_direct_phi_history   = deque(maxlen=3600)

        # Fade / pause state
        self.fade           = None
        self.faded_states   = None
        self.last_states    = None
        self.paused         = False
        self.stopping        = False
        self.decel_progress  = 0.0
        self.resuming        = False
        self.resume_progress = 0.0
        self.decel_duration  = 0.5   # seconds shared by both stop and resume ramps

        # Homing state machine
        # States: None | 'decelling' | 'waiting' | 'resuming'
        self.homing_state         = None
        self._homing_trigger_op   = None    # op name whose par gets pulsed to start homing
        self._homing_trigger_par  = None    # parameter name on that op
        self._homing_serial_dat   = None    # Serial DAT op name to poll for done string
        self._homing_done_string  = 'HOMED' # text sequence that signals homing complete
        self._saved_phi           = None    # phi values saved when decel completes

        # Optional CHOP to drive quaternion rotation axis each frame
        self._rotation_axis_chop  = 'Rotation_Vector'    # op name; expects channels x, y, z

        self._behavior_fns = {
            'sine':                 self._sine,
            'wave':                 self._wave,
            'noise':                self._noise,
            'circular':             self._circular,
            'quaternion':           self._quaternion,
            'circle_zy':            self._circle_zy,
            'figure8':              self._figure8,
            'figure9':              self._figure9,
            'flip':                 self._flip,
            'stepper_speed_control':  self._stepper_speed_control,
            'stepper_direct_control': self._stepper_direct_control,
            'home_sweep':             self._home_sweep,
        }

    # -------------------------
    # Update loop
    # -------------------------

    def Update(self):
        current_time = time.time()
        dt = current_time - self.previous_time
        if dt < 0.001:   # skip spurious double-cooks within the same TD frame
            return
        self.previous_time = current_time

        # Pull rotation axis from CHOP only while quaternion behavior is active
        if self._rotation_axis_chop and self.weights.get('quaternion', 0.0) > 0.0:
            try:
                chop = self.ownerComp.op(self._rotation_axis_chop) \
                       or self.ownerComp.parent().op(self._rotation_axis_chop)
                if chop:
                    self.behaviors['quaternion']['rotation_axis'] = [
                        chop['x'][0], chop['y'][0], chop['z'][0]
                    ]
            except Exception:
                pass

        # Homing state transitions
        if self.homing_state == 'decelling' and self.paused and not self.stopping:
            # Save phi at the moment we fully stop, before hardware homes
            self._saved_phi = [s['phi'] for s in self.last_states] if self.last_states else [90.0] * len(self.robots)
            self.homing_state = 'waiting'
            self._sendHomingTrigger()
        elif self.homing_state == 'waiting':
            if self._checkSerialForHomingDone():
                self._completeHoming()
                return
        elif self.homing_state == 'resuming' and not self.resuming and not self.paused:
            self.homing_state = None

        if self.paused:
            return

        if self.stopping:
            self.decel_progress = min(1.0, self.decel_progress + dt / self.decel_duration)
            scale = 1.0 - self.decel_progress
            if scale <= 0.0:
                self.stopping = False
                self.paused   = True
                for r in self.robots:
                    r.Halt()
                return
            effective_dt = dt * scale
        elif self.resuming:
            self.resume_progress = min(1.0, self.resume_progress + dt / self.decel_duration)
            scale = self.resume_progress
            if scale >= 1.0:
                self.resuming    = False
                scale            = 1.0
            effective_dt = dt * scale
        else:
            effective_dt = dt

        self.time += effective_dt
        self._updateFade(effective_dt)
        self._smoothBehaviorParameters(effective_dt)

        final_states = self._blendAllBehaviors(effective_dt)

        # Faded states override blended states while a fade is in progress
        if self.fade and self.faded_states:
            final_states = self.faded_states

        if final_states:
            self._applyStates(final_states, dt)

    def _blendAllBehaviors(self, dt):
        """Per-robot normalized weighted blend. robot_mask scales each behavior's contribution per robot."""
        active = [(name, self.weights[name])
                  for name in self._behavior_fns
                  if self.weights.get(name, 0.0) > 0.0]
        if not active:
            return None

        n = len(self.robots)

        # Evaluate all active behaviors once
        behavior_data = []
        for name, w in active:
            states = self._behavior_fns[name](dt)
            if states is None:
                continue
            mask = self.robot_masks.get(name, None)
            if not mask or len(mask) < n:
                mask = [1.0] * n
            behavior_data.append((states, w, mask))

        if not behavior_data:
            return None

        # Build per-robot blended result — None means "no command, skip this robot"
        final_states = [None] * n
        last = self.last_states

        for i in range(n):
            robot_total = sum(w * m[i] for _, w, m in behavior_data)
            if robot_total == 0.0:
                # Masked robot — hold last known state, or send no command if no history
                if last and i < len(last) and last[i] is not None:
                    s = last[i]
                    final_states[i] = {'theta': s['theta'], 'phi': s['phi'], 'led': list(s['led'])}
                continue
            final_states[i] = {'theta': 0.0, 'phi': 0.0, 'led': [0.0, 0.0, 0.0]}
            for states, w, m in behavior_data:
                nw = (w * m[i]) / robot_total
                final_states[i]['theta'] += states[i]['theta'] * nw
                final_states[i]['phi']   += states[i]['phi']   * nw
                for j in range(3):
                    final_states[i]['led'][j] += states[i]['led'][j] * nw

        return final_states

    # -------------------------
    # Behaviors
    # -------------------------
    # Each behavior has signature: (self, dt) -> list[state_dict] | None
    # state_dict: {'theta': float, 'phi': float, 'led': [r, g, b]}

    def _sine(self, dt):
        """Theta rotates at constant velocity; phi oscillates sinusoidally per robot."""
        p = self.smoothed_params['sine']
        states = []
        for i, _ in enumerate(self.robots):
            self.phase_phi[i] = (
                self.phase_phi[i] + 2 * math.pi * p['frequency_phi'] * dt
            ) % (2 * math.pi)
            self.sine_theta_positions[i] = (
                self.sine_theta_positions[i] + p['theta_velocity'] * 360.0 * dt
            ) % 360.0

            theta = (self.sine_theta_positions[i] + i * p['phase_theta']) % 360.0
            phi   = math.sin(self.phase_phi[i] + i * p['phase_shift']) * p['amplitude_phi'] + p['bias_phi']
            states.append({'theta': theta, 'phi': phi, 'led': [1.0, 1.0, 1.0]})
        return states

    def _wave(self, dt):
        """Both theta and phi oscillate sinusoidally using absolute time."""
        p = self.smoothed_params['wave']
        states = []
        for i, _ in enumerate(self.robots):
            phase = i * p['phase_shift']
            theta = math.sin(self.time * p['frequency_theta'] + i * 0.5 + phase) * p['amplitude_theta'] + p['bias_theta']
            phi   = math.sin(self.time * p['frequency_phi']   + i * 0.5 + phase) * p['amplitude_phi']   + p['bias_phi']
            states.append({'theta': theta % 360.0, 'phi': phi, 'led': [0.0, 0.0, 1.0]})
        return states

    def _noise(self, dt):
        """Higher-frequency wave variant with smaller amplitude."""
        p = self.smoothed_params['noise']
        states = []
        for i, _ in enumerate(self.robots):
            phase = i * p['phase_shift']
            theta = math.sin(self.time * p['frequency_theta'] + i + phase) * p['amplitude_theta'] + p['bias_theta']
            phi   = math.sin(self.time * p['frequency_phi']   + i + phase) * p['amplitude_phi']   + p['bias_phi']
            states.append({'theta': theta % 360.0, 'phi': phi, 'led': [1.0, 0.0, 0.0]})
        return states

    def _circular(self, dt):
        """Circular motion in a vertical plane tilted from Y axis."""
        p = self.smoothed_params['circular']
        states = []
        for i, _ in enumerate(self.robots):
            self.circular_angles[i] += math.radians(p['angular_speed']) * dt
            angle = self.circular_angles[i] + i * p['phase_shift']
            theta = (p['azimuth_offset'] + p['amplitude'] * math.cos(angle)) % 360.0
            phi   = p['tilt_angle']      + p['amplitude'] * math.sin(angle)
            states.append({'theta': theta, 'phi': phi, 'led': [0.5, 0.5, 0.5]})
        return states

    def _quaternion(self, dt):
        """
        Rotate each robot's arm in a plane perpendicular to rotation_axis.

        Each frame the stored unit-vector arm position is rotated around the axis
        by angular_speed * dt (Rodrigues' formula). Robots are spread by phase_shift.
        Set the axis each frame via setRotationAxis(x, y, z).
        """
        p    = self.smoothed_params['quaternion']
        axis = p['rotation_axis']

        if self.quaternion_positions is None:
            self._initQuaternionPositions(axis, p['phase_shift'])

        angle_step = p['angular_speed'] * 2.0 * math.pi * dt
        states = []
        for i, _ in enumerate(self.robots):
            v = self._rotate(self.quaternion_positions[i], axis, angle_step)
            n = math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
            if n > 1e-9:
                v = (v[0]/n, v[1]/n, v[2]/n)
            self.quaternion_positions[i] = v
            theta, phi = self._xyzToSpherical(v)
            states.append({'theta': theta, 'phi': phi, 'led': [0.0, 1.0, 1.0]})
        return states

    def _circle_zy(self, dt):
        """
        Trace a circle on the unit sphere that lies in the plane x = x_offset.
        Center of circle: (x_offset, 0, 0).  Radius: sqrt(1 - x_offset²).
        Points: v = (x_offset, r·cos(angle), r·sin(angle)).
        """
        p   = self.smoothed_params['circle_zy']
        x   = max(-0.999, min(0.999, p['x_offset']))
        r   = math.sqrt(1.0 - x * x)
        states = []
        for i, _ in enumerate(self.robots):
            self.circle_zy_angles[i] = (
                self.circle_zy_angles[i] + p['angular_speed'] * 2.0 * math.pi * dt
            ) % (2.0 * math.pi)
            angle = self.circle_zy_angles[i] + i * p['phase_shift']
            v = (x, r * -math.cos(angle), r * math.sin(angle))
            theta, phi = self._xyzToSpherical(v)
            states.append({'theta': theta, 'phi': phi, 'led': [0.8, 0.2, 1.0]})
        return states

    def _figure8(self, dt):
        """
        Lissajous figure-8: theta oscillates at frequency f, phi at 2f.
        Phase is accumulated each frame so frequency changes are seamless.
        """
        p = self.smoothed_params['figure8']
        states = []
        for i, _ in enumerate(self.robots):
            self.figure8_phases[i] = (
                self.figure8_phases[i] + 2.0 * math.pi * p['frequency'] * dt
            ) % (2.0 * math.pi)
            phase = self.figure8_phases[i] + i * p['phase_shift']
            theta = p['bias_theta'] + p['amplitude_theta'] * math.sin(phase + p['theta_phase_shift'])
            phi   = p['bias_phi']   + p['amplitude_phi']   * math.sin(2.0 * phase)
            states.append({'theta': theta % 360.0, 'phi': phi, 'led': [1.0, 0.5, 0.0]})
        return states

    def _figure9(self, dt):
        """
        Lissajous figure-9: theta oscillates at 2f, phi at f (inverse of figure8).
        Phase accumulated per frame for smooth frequency transitions.
        """
        p = self.smoothed_params['figure9']
        states = []
        for i, _ in enumerate(self.robots):
            self.figure9_phases[i] = (
                self.figure9_phases[i] + 2.0 * math.pi * p['frequency'] * dt
            ) % (2.0 * math.pi)
            phase = self.figure9_phases[i] + i * p['phase_shift']
            theta = p['bias_theta'] + p['amplitude_theta'] * math.sin(2.0 * phase + p['theta_phase_shift'])
            phi   = p['bias_phi']   + p['amplitude_phi']   * math.sin(phase)
            states.append({'theta': theta % 360.0, 'phi': phi, 'led': [0.5, 1.0, 0.0]})
        return states

    def _flip(self, _dt):
        """Set all robots to phi=-135 (state=0) or phi=135 (state=1)."""
        p   = self.smoothed_params['flip']
        phi = 135.0 if p['state'] >= 0.5 else -135.0
        return [
            {'theta': p['bias_theta'], 'phi': phi, 'led': [1.0, 1.0, 0.0]}
            for _ in self.robots
        ]

    def _stepper_speed_control(self, dt):
        """
        Theta (stepper) rotates continuously at stepper_speed rps; robot i's theta is offset by
        i * stepper_phase_offset degrees.  Phi is set directly from a time-delayed phi_position:
        robot i reads the value from i * phi_delay seconds ago, creating a cascading wave.
        """
        p = self.smoothed_params['stepper_speed_control']
        self._stepper_speed_phi_history.append((self.time, p['phi_position']))
        states = []
        for i, _ in enumerate(self.robots):
            self.stepper_speed_theta[i] = (
                self.stepper_speed_theta[i] + p['stepper_speed'] * 360.0 * dt
            ) % 360.0
            theta = (self.stepper_speed_theta[i] + i * p['stepper_phase_offset']) % 360.0
            phi = max(-135.0, min(135.0,
                      self._get_delayed_value(self._stepper_speed_phi_history, i * p['phi_delay'])))
            states.append({'theta': theta, 'phi': phi, 'led': [1.0, 0.0, 1.0]})
        return states

    def _stepper_direct_control(self, _dt):
        """
        Direct position control for both theta and phi with independent cascading delays.
        Robot i reads stepper_position / phi_position from i * stepper_delay / i * phi_delay
        seconds ago respectively.
        """
        p = self.smoothed_params['stepper_direct_control']
        self._stepper_direct_theta_history.append((self.time, p['stepper_position']))
        self._stepper_direct_phi_history.append((self.time, p['phi_position']))
        states = []
        for i, _ in enumerate(self.robots):
            theta = self._get_delayed_value(self._stepper_direct_theta_history, i * p['stepper_delay']) % 360.0
            phi   = max(-135.0, min(135.0,
                        self._get_delayed_value(self._stepper_direct_phi_history, i * p['phi_delay'])))
            states.append({'theta': theta, 'phi': phi, 'led': [0.0, 1.0, 0.5]})
        return states

    def _home_sweep(self, dt):
        """Sweep all robots: theta from theta_start down to 0° at theta_speed rps,
        phi from each robot's current position toward phi_target at phi_speed deg/s."""
        p           = self.smoothed_params['home_sweep']
        theta_start = p['theta_start'] if isinstance(p['theta_start'], (int, float)) else 135.0
        theta_speed = p['theta_speed'] if isinstance(p['theta_speed'], (int, float)) else 0.3
        phi_target  = p['phi_target']  if isinstance(p['phi_target'],  (int, float)) else 0.0
        phi_speed   = p['phi_speed']   if isinstance(p['phi_speed'],   (int, float)) else 45.0

        if self.home_sweep_theta is None:
            self.home_sweep_theta = [float(theta_start)] * len(self.robots)
        if self.home_sweep_phi is None:
            last = self.last_states or [None] * len(self.robots)
            self.home_sweep_phi = [
                last[i]['phi'] if (i < len(last) and last[i]) else 0.0
                for i in range(len(self.robots))
            ]

        states = []
        phi_step = phi_speed * dt
        for i, _ in enumerate(self.robots):
            self.home_sweep_theta[i] = max(0.0, self.home_sweep_theta[i] - theta_speed * 360.0 * dt)
            self.home_sweep_phi[i] += max(-phi_step, min(phi_step, phi_target - self.home_sweep_phi[i]))
            states.append({'theta': self.home_sweep_theta[i], 'phi': self.home_sweep_phi[i], 'led': [0.0, 0.0, 0.0]})
        return states

    def resetHomeSweep(self):
        """Reset home_sweep accumulators so the next activation starts fresh."""
        self.home_sweep_theta = None
        self.home_sweep_phi   = None

    def _get_delayed_value(self, history, delay_seconds):
        """Return the value recorded delay_seconds ago from the time-stamped history deque."""
        if not history:
            return 0.0
        target_time = self.time - delay_seconds
        if target_time <= history[0][0]:
            return history[0][1]
        if target_time >= history[-1][0]:
            return history[-1][1]
        for j in range(len(history) - 1, -1, -1):
            if history[j][0] <= target_time:
                return history[j][1]
        return history[0][1]

    def _initQuaternionPositions(self, axis, phase_shift):
        """Place each robot evenly on the circle perpendicular to the rotation axis."""
        ax, ay, az = axis
        n = math.sqrt(ax*ax + ay*ay + az*az)
        if n < 1e-9:
            ax, ay, az = 0.0, 1.0, 0.0
        else:
            ax, ay, az = ax/n, ay/n, az/n
        # Find a reference vector perpendicular to axis via Gram-Schmidt
        if abs(ax) < 0.9:
            rx, ry, rz = 1.0, 0.0, 0.0
        else:
            rx, ry, rz = 0.0, 1.0, 0.0
        dot = rx*ax + ry*ay + rz*az
        rx -= dot*ax;  ry -= dot*ay;  rz -= dot*az
        n2 = math.sqrt(rx*rx + ry*ry + rz*rz)
        rx, ry, rz = rx/n2, ry/n2, rz/n2
        self.quaternion_positions = [
            self._rotate((rx, ry, rz), (ax, ay, az), i * phase_shift)
            for i in range(len(self.robots))
        ]

    @staticmethod
    def _rotate(v, axis, angle_rad):
        """Rodrigues' rotation formula: rotate vector v around axis by angle_rad."""
        ax, ay, az = axis
        n = math.sqrt(ax*ax + ay*ay + az*az)
        if n < 1e-9:
            return v
        ax, ay, az = ax/n, ay/n, az/n
        vx, vy, vz = v
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        dot   = ax*vx + ay*vy + az*vz
        cx    = ay*vz - az*vy
        cy    = az*vx - ax*vz
        cz    = ax*vy - ay*vx
        return (
            vx*cos_a + cx*sin_a + ax*dot*(1.0 - cos_a),
            vy*cos_a + cy*sin_a + ay*dot*(1.0 - cos_a),
            vz*cos_a + cz*sin_a + az*dot*(1.0 - cos_a),
        )

    @staticmethod
    def _xyzToSpherical(v):
        """
        Convert a unit xyz vector to (theta_deg, phi_deg).

        Matches the QuaternionScriptCallback coordinate system:
          y = cos(phi)  →  phi=0 at Y+ (arm up), phi=90 at XZ plane
          theta = atan2(x, z)
        """
        vx, vy, vz = v
        phi   = math.degrees(math.acos(max(-1.0, min(1.0, vy))))
        theta = math.degrees(math.atan2(vx, vz)) % 360.0
        return theta, phi

    # -------------------------
    # Interpolation helpers
    # -------------------------

    @staticmethod
    def _easeInOutCubic(t):
        if t < 0.5:
            return 4 * t * t * t
        t2 = 2 * t - 2
        return 0.5 * t2 * t2 * t2 + 1

    @staticmethod
    def _shortestThetaDelta(target, source, wrap=360.0):
        return ((target - source + wrap * 0.5) % wrap) - wrap * 0.5

    def _interpolateStates(self, start_states, end_states, t):
        result = []
        for s, e in zip(start_states, end_states):
            td    = self._shortestThetaDelta(e['theta'], s['theta'])
            theta = (s['theta'] + td * t) % 360.0
            phi   = s['phi'] + (e['phi'] - s['phi']) * t
            led   = [s['led'][j] + (e['led'][j] - s['led'][j]) * t for j in range(3)]
            result.append({'theta': theta, 'phi': phi, 'led': led})
        return result

    def _smoothBehaviorParameters(self, dt):
        if dt <= 0:
            return
        alpha = 1.0 - math.exp(-self.parameter_smoothing_speed * dt)
        for name, target in self.behaviors.items():
            current = self.smoothed_params[name]
            for key, tv in target.items():
                if isinstance(tv, (int, float)):
                    cv = current.get(key, tv)
                    if isinstance(cv, (int, float)):
                        current[key] = cv + (tv - cv) * alpha
                    else:
                        current[key] = tv  # cv was a non-numeric (e.g. string) — snap to target
                else:
                    current[key] = tv  # non-numeric params pass through unchanged

    # -------------------------
    # Apply to robots
    # -------------------------

    def _applyStates(self, states, dt):
        for i, (r, s) in enumerate(zip(self.robots, states)):
            if s is None:
                continue  # masked robot with no history — send no command
            theta = (s['theta'] * self.theta_directions[i]) % 360.0
            r.SetState(theta, s['phi'], s['led'], dt)
            r.PushToCHOP()
        self.last_states = states

    def _getCurrentStates(self):
        return self.last_states or [
            {'theta': 0.0, 'phi': 90.0, 'led': [0.0, 0.0, 0.0]}
            for _ in self.robots
        ]

    # -------------------------
    # Fade system
    # -------------------------

    def fadeTo(self, target, duration=2.0):
        if target not in self._behavior_fns:
            return
        self.fade = {
            'start_states': self._getCurrentStates(),
            'end_states':   self._computeTargetStates(target, duration),
            'time':         0.0,
            'duration':     duration,
            'target':       target,
        }

    def _updateFade(self, dt):
        if not self.fade:
            return
        f = self.fade
        f['time'] += dt
        t = min(f['time'] / f['duration'], 1.0)
        if t >= 1.0:
            self.zeroAll()
            self.weights[f['target']] = 1.0
            self.fade = self.faded_states = None
        else:
            self.faded_states = self._interpolateStates(
                f['start_states'], f['end_states'], self._easeInOutCubic(t)
            )

    def _computeTargetStates(self, target, duration):
        """
        Predict where a behavior will be `duration` seconds from now.
        Used to set the fade end-point so the transition lands seamlessly.
        """
        if target == 'sine':
            return self._computeSineStates(duration)
        if target == 'circular':
            return self._computeCircularStates(duration)
        # Stateless behaviors: just sample them at current time
        fn = self._behavior_fns.get(target)
        if fn: 
            result = fn(0.0)
            if result:
                return result
        return self._getCurrentStates()

    def _computeSineStates(self, duration=0.0):
        """Predict sine positions after `duration` seconds (for fade targeting)."""
        p = self.behaviors['sine']
        states = []
        for i, _ in enumerate(self.robots):
            theta = (
                (self.sine_theta_positions[i] + p['theta_velocity'] * 360.0 * duration)
                + i * p['phase_theta']
            ) % 360.0
            phi = (
                math.sin((self.time + duration) * p['frequency_phi'] + i * p['phase_shift'])
                * p['amplitude_phi'] + p['bias_phi']
            )
            states.append({'theta': theta, 'phi': phi, 'led': [1.0, 1.0, 1.0]})
        return states

    def _computeCircularStates(self, duration=0.0):
        """Predict circular positions after `duration` seconds (for fade targeting)."""
        p = self.behaviors['circular']
        states = []
        for i, _ in enumerate(self.robots):
            angle = self.circular_angles[i] + math.radians(p['angular_speed']) * duration + i * p['phase_shift']
            theta = (p['azimuth_offset'] + p['amplitude'] * math.cos(angle)) % 360.0
            phi   = p['tilt_angle']      + p['amplitude'] * math.sin(angle)
            states.append({'theta': theta, 'phi': phi, 'led': [0.5, 0.5, 0.5]})
        return states
    
    def _clamp(self,value, min_value, max_value):
        """Clamp a value between min_value and max_value."""
        return max(min_value, min(max_value, value))


    # -------------------------
    # Parameter controls
    # -------------------------
    def getThetaVelocity(self):
        return self.behaviors['sine']['theta_velocity']
    
    def setWeight(self, name, weight):
        if name in self.weights:
            self.weights[name] = weight

    def setParam(self, behavior, key, value):
        """Generic parameter setter. e.g. setParam('sine', 'theta_velocity', 45.0)"""
        if behavior in self.behaviors and key in self.behaviors[behavior]:
            self.behaviors[behavior][key] = value

    def setFrequency(self, name, frequency_theta=None, frequency_phi=None):
        if name not in self.behaviors:
            return
        p = self.behaviors[name]
        if frequency_theta is not None and 'frequency_theta' in p:
            p['frequency_theta'] = frequency_theta
        if frequency_phi is not None and 'frequency_phi' in p:
            p['frequency_phi'] = frequency_phi

    def setThetaVelocity(self, velocity):
        self.behaviors['sine']['theta_velocity'] = velocity

    def setPhaseShift(self, name, phase_shift):
        if name in self.behaviors:
            self.behaviors[name]['phase_shift'] = phase_shift

    def setBias(self, name, bias_theta=None, bias_phi=None):
        if name not in self.behaviors:
            return
        p = self.behaviors[name]
        if bias_theta is not None:
            key = 'phase_theta' if name == 'sine' else 'bias_theta'
            if key in p:
                p[key] = bias_theta
        if bias_phi is not None and 'bias_phi' in p:
            p['bias_phi'] = bias_phi

    def setCircularSpeed(self, angular_speed):
        self.behaviors['circular']['angular_speed'] = angular_speed

    def setCircularTilt(self, tilt_angle):
        if tilt_angle >= 45.0:
            self.behaviors['circular']['tilt_angle'] = tilt_angle

    def setCircularAzimuth(self, azimuth_offset):
        self.behaviors['circular']['azimuth_offset'] = azimuth_offset % 360.0

    def setRotationAxis(self, x, y, z):
        """Set the 3D rotation axis for the quaternion behavior. Arms move perpendicular to this vector."""
        self.behaviors['quaternion']['rotation_axis'] = [x, y, z]

    def setRotationAxisCHOP(self, op_name):
        """
        Connect a CHOP as the live rotation axis source.
        The CHOP must have channels named x, y, z.
        Pass None to disconnect.
        """
        self._rotation_axis_chop = op_name

    def setQuaternionSpeed(self, angular_speed):
        """Set rotation speed (rps) for the quaternion behavior."""
        #angular_speed = self._clamp(angular_speed, -1.5, 1.5)  # limit to reasonable max
        self.behaviors['quaternion']['angular_speed'] = angular_speed
    
    def setQuaternionPhaseShift(self, phase_shift):
        """Set the phase offset (rad) between robots for the quaternion behavior."""
        self.behaviors['quaternion']['phase_shift'] = phase_shift
        self.quaternion_positions = None  # reinitialize with new spacing

    def setCircleZYOffset(self, x_offset):
        """Set the X plane position for circle_zy (-1 < x < 1). x=0 → great circle, x=0.5 → tilted circle."""
        self.behaviors['circle_zy']['x_offset'] = max(-0.999, min(0.999, x_offset))

    def setCircleZYSpeed(self, angular_speed):
        """Set rotation speed (rps) for circle_zy behavior."""
        self.behaviors['circle_zy']['angular_speed'] = angular_speed

    def setFlipState(self, state):
        """Flip all robots: 0 → phi=-135, 1 → phi=135."""
        self.behaviors['flip']['state'] = 1.0 if state else 0.0

    def setStepperSpeed(self, stepper_speed):
        """Set theta rotation speed (rps) for stepper_speed_control."""
        self.behaviors['stepper_speed_control']['stepper_speed'] = stepper_speed

    def setStepperPhaseOffset(self, stepper_phase_offset):
        """Set per-robot theta offset (deg) for stepper_speed_control. Robot i += i * offset."""
        self.behaviors['stepper_speed_control']['stepper_phase_offset'] = stepper_phase_offset

    def setStepperSpeedPhi(self, phi_position):
        """Set phi setpoint (deg, -135 to 135) for stepper_speed_control."""
        self.behaviors['stepper_speed_control']['phi_position'] = max(-135.0, min(135.0, phi_position))

    def setStepperSpeedPhiDelay(self, phi_delay):
        """Set cascading phi delay (s per robot) for stepper_speed_control."""
        self.behaviors['stepper_speed_control']['phi_delay'] = max(0.0, phi_delay)

    def setStepperDirectPosition(self, stepper_position):
        """Set theta setpoint (deg) for stepper_direct_control."""
        self.behaviors['stepper_direct_control']['stepper_position'] = stepper_position % 360.0

    def setStepperDirectDelay(self, stepper_delay):
        """Set cascading theta delay (s per robot) for stepper_direct_control."""
        self.behaviors['stepper_direct_control']['stepper_delay'] = max(0.0, stepper_delay)

    def setStepperDirectPhi(self, phi_position):
        """Set phi setpoint (deg, -135 to 135) for stepper_direct_control."""
        self.behaviors['stepper_direct_control']['phi_position'] = max(-135.0, min(135.0, phi_position))

    def setStepperDirectPhiDelay(self, phi_delay):
        """Set cascading phi delay (s per robot) for stepper_direct_control."""
        self.behaviors['stepper_direct_control']['phi_delay'] = max(0.0, phi_delay)

    def setRobotMask(self, behavior, mask):
        """
        Set which robots a behavior drives.
        mask — list/tuple of 6 values; 1.0 = active, 0.0 = inactive.
        Example: setRobotMask('sine', [1,1,1,0,0,0])  → only robots 1-3
        """
        if behavior not in self.robot_masks:
            return
        self.robot_masks[behavior] = [float(v) for v in mask]

    def zeroAll(self):
        for name in self.weights:
            self.weights[name] = 0.0

    # -------------------------
    # Playback controls
    # -------------------------
    def hardStop(self):
        """Immediately zero velocity and push 100000 to all robot CHOPs. No ramp."""
        self.paused          = True
        self.stopping        = False
        self.resuming        = False
        self.decel_progress  = 0.0
        self.resume_progress = 0.0
        self.homing_state    = None
        self.fade            = None
        self.faded_states    = None
        for r in self.robots:
            r.Halt()

    def stop(self):
        """Decelerate all robots to a halt over decel_duration seconds. Safe to call repeatedly."""
        self.stopping       = True
        self.decel_progress = 0.0
        self.paused         = False   # ensure Update() runs the decel loop
        self.fade           = None
        self.faded_states   = None

    def setDecelDuration(self, seconds):
        """Set how long the stop/resume ramp takes (default 0.5 s)."""
        self.decel_duration = max(0.5, seconds)

    # -------------------------
    # Homing sequence
    # -------------------------

    def startHoming(self, trigger_op=None, trigger_par='Pulse',
                    serial_dat=None, done_string='HOMED'):
        """
        Begin the homing sequence:
          1. Decelerate to a stop (uses decel_duration).
          2. Pulse trigger_op.trigger_par to start hardware homing.
          3. Poll serial_dat each frame for done_string.
          4. On match: snap all robots to theta=315, restore saved phi, ramp back up.

        trigger_op  — op name to pulse for homing trigger. None to skip.
        trigger_par — parameter name on that op (default 'Pulse').
        serial_dat  — Serial DAT op name to read. None disables auto-detection;
                      call onHomingComplete() manually instead.
        done_string — text to look for in serial_dat rows (default 'HOMED').
        """
        self.homing_state         = 'decelling'
        self._homing_trigger_op   = trigger_op
        self._homing_trigger_par  = trigger_par
        self._homing_serial_dat   = serial_dat
        self._homing_done_string  = done_string
        self.stop()

    def startMotorHoming(self, phi_target=None):
        """Trigger motor homing: toggle button2. If phi_target is not None, snap all robots' phi first."""
        if phi_target is not None:
            phi_target = max(-135.0, min(135.0, float(phi_target)))
            last = self.last_states or [None] * len(self.robots)
            for i, r in enumerate(self.robots):
                theta = last[i]['theta'] if (i < len(last) and last[i]) else 0.0
                r.SetState(theta, phi_target, [0.0, 0.0, 0.0], None)
                r.PushToCHOP()
        try:
            b = op('/project1/Robot_Data_to_Arduino/button2')
            b.par.value0 = 0 if b.par.value0 == 1 else 1
        except Exception as e:
            print(f'[RobotController] motor homing trigger failed: {e}')

    def onHomingComplete(self):
        """Manually signal homing done (use when not using serial_dat polling)."""
        if self.homing_state == 'waiting':
            self._completeHoming()

    def _completeHoming(self):
        """Snap robots to theta=315, restore saved phi, begin resume ramp."""
        saved_phi = self._saved_phi or [90.0] * len(self.robots)
        # Reset sine theta accumulators so the behavior continues from 315
        for i in range(len(self.robots)):
            self.sine_theta_positions[i] = 315.0
        # Set each robot's physical position before the ramp starts
        for i, r in enumerate(self.robots):
            r.SetState(315.0, saved_phi[i], [0.0, 0.0, 0.0], None)
            r.PushToCHOP()
        self.homing_state = 'resuming'
        self.resume()

    def _checkSerialForHomingDone(self):
        if not self._homing_serial_dat:
            return False
        try:
            dat = self.ownerComp.op(self._homing_serial_dat) \
                  or self.ownerComp.parent().op(self._homing_serial_dat)
            if not dat:
                return False
            for row in range(dat.numRows):
                if self._homing_done_string in str(dat[row, 0]):
                    return True
        except Exception:
            pass
        return False

    def _sendHomingTrigger(self):
        if not self._homing_trigger_op:
            return
        try:
            o = self.ownerComp.op(self._homing_trigger_op) \
                or self.ownerComp.parent().op(self._homing_trigger_op)
            if o:
                getattr(o.par, self._homing_trigger_par).pulse()
        except Exception as e:
            print(f'[RobotController] homing trigger failed: {e}')

    def setMaxThetaAcceleration(self, accel):
        """Global theta acceleration limit (rps/s). 0 = unlimited."""
        for r in self.robots:
            r.setMaxThetaAcceleration(accel)

    def setMaxPhiAcceleration(self, accel):
        """Global phi acceleration limit (deg/s²). 0 = unlimited."""
        for r in self.robots:
            r.setMaxPhiAcceleration(accel)

    def play(self):
        """Alias for resume(). Ramp velocity up from zero over decel_duration seconds."""
        self.resume()

    def resume(self):
        """Ramp velocity up from zero over decel_duration seconds."""
        self.paused          = False
        self.stopping        = False
        self.decel_progress  = 0.0
        self.resuming        = True
        self.resume_progress = 0.0

    def servo_min(self):
        """Move all robots to their minimum position (phi = -135°)."""
        states = [{'theta': 0.0, 'phi': -135.0, 'led': [0.0, 0.0, 0.0]} for _ in self.robots]
        self._applyStates(states, 0.0)
        self.paused       = True
        self.fade         = None
        self.faded_states = None
        self.last_states  = states

    def servo_max(self):
        """Move all robots to phi = 0° (pointing along Y+)."""
        states = [{'theta': 0.0, 'phi': 135.0, 'led': [0.0, 0.0, 0.0]} for _ in self.robots]
        self._applyStates(states, 0.0)
        self.paused       = True
        self.fade         = None
        self.faded_states = None
        self.last_states  = states

    def servo_zero(self):
        """Move all robots to phi = 0° (pointing along Y+)."""
        states = [{'theta': 0.0, 'phi': 0.0, 'led': [0.0, 0.0, 0.0]} for _ in self.robots]
        self._applyStates(states, 0.0)
        self.paused       = True
        self.fade         = None
        self.faded_states = None
        self.last_states  = states
 
    # -------------------------
    # Dynamic behavior registration (used by MotionRecorderEXT)
    # -------------------------

    def registerBehavior(self, name, fn, default_params=None):
        """Register a dynamic behavior (e.g. a recorded clip) so it can be weighted and blended."""
        self._behavior_fns[name]   = fn
        self.weights[name]         = 0.0
        self.robot_masks[name]     = [1.0] * 6
        self.behaviors[name]       = default_params or {}
        self.smoothed_params[name] = dict(self.behaviors[name])

    def unregisterBehavior(self, name):
        """Remove a dynamically registered behavior."""
        for d in (self._behavior_fns, self.weights, self.robot_masks, self.behaviors, self.smoothed_params):
            d.pop(name, None)

    # -------------------------
    # Output
    # -------------------------

    def GetAllStates(self):
        return [r.GetState() for r in self.robots]

    def ExposedMethod(self):
        print('ExposedMethod called!')
