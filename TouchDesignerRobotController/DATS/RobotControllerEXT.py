import math
import time


class RobotControllerEXT:
    """
    Owns all motion logic for 6 robot arms — behaviors, blending, fading, and
    applying states to individual RobotEXT instances each frame.

    Adding a new behavior (e.g. 'quaternion') requires exactly three steps:
      1. Add its default parameters to BEHAVIOR_DEFAULTS below.
      2. Implement a _<name>(self, dt) method that returns a states list.
      3. Register it in self._behavior_fns inside __init__.
    """

    # -------------------------
    # Behavior default parameters
    # Each key maps to the parameters used by that behavior's compute method.
    # Weights live separately in self.weights — they are NOT stored here.
    # -------------------------

    BEHAVIOR_DEFAULTS = {
        'sine': {
            'theta_velocity': 30.0,   # deg/s constant rotation speed
            'frequency_phi':   0.5,   # Hz
            'phase_shift':     0.0,   # rad offset between robots (phi)
            'phase_theta':     0.0,   # deg offset between robots (theta)
            'bias_phi':       -90.0,   # deg — phi rest position
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
            'angular_speed': 30.0,    # deg/s angular rotation speed
            'tilt_angle':    45.0,    # deg from Y axis (min 45)
            'azimuth_offset': 0.0,    # deg in XZ plane (0-360)
            'phase_shift':   0.0,     # rad offset between robots
        },
        # Step 1 — quaternion defaults:
        # quaternions is a list of (qx, qy, qz, qw) tuples, one per robot.
        # Set via setQuaternion(robot_index, qx, qy, qz, qw) or setAllQuaternions([...]).
        'quaternion': {
            'quaternions': None,
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
        self.weights['sine'] = 1.0

        # Target parameter values (what the user sets)
        self.behaviors = {name: dict(defaults) for name, defaults in self.BEHAVIOR_DEFAULTS.items()}
        # Smoothed parameter values (lag toward targets — what behaviors actually read)
        self.smoothed_params = {name: dict(defaults) for name, defaults in self.BEHAVIOR_DEFAULTS.items()}
        self.parameter_smoothing_speed = 6.0

        # Per-robot phase accumulators for stateful behaviors
        self.sine_theta_positions = [0.0] * len(self.robots)
        self.phase_phi            = [0.0] * len(self.robots)
        self.theta_directions     = [1]   * len(self.robots)
        self.circular_angles      = [0.0] * len(self.robots)

        # Fade / pause state
        self.fade         = None
        self.faded_states = None
        self.last_states  = None
        self.paused       = False

        # Step 3 — behavior dispatch table.
        # Register new behaviors here alongside steps 1 and 2.
        self._behavior_fns = {
            'sine':       self._sine,
            'wave':       self._wave,
            'noise':      self._noise,
            'circular':   self._circular,
            'quaternion': self._quaternion,
        }

    # -------------------------
    # Update loop
    # -------------------------

    def Update(self):
        current_time = time.time()
        dt = current_time - self.previous_time
        self.previous_time = current_time
        self.time += dt

        if self.paused and self.last_states:
            self._applyStates(self.last_states, dt)
            return

        self._updateFade(dt)
        self._smoothBehaviorParameters(dt)

        final_states = self._blendAllBehaviors(dt)

        # Faded states override blended states while a fade is in progress
        if self.fade and self.faded_states:
            final_states = self.faded_states

        if final_states:
            self._applyStates(final_states, dt)

    def _blendAllBehaviors(self, dt):
        """Normalized weighted blend across all active (weight > 0) behaviors."""
        active = [(name, self.weights[name])
                  for name in self._behavior_fns
                  if self.weights.get(name, 0.0) > 0.0]
        if not active:
            return None

        total_weight = sum(w for _, w in active)
        if total_weight == 0.0:
            return None

        final_states = None
        for name, w in active:
            states = self._behavior_fns[name](dt)
            if states is None:
                continue
            nw = w / total_weight  # normalized so all weights sum to 1
            if final_states is None:
                final_states = [
                    {'theta': s['theta'] * nw,
                     'phi':   s['phi']   * nw,
                     'led':   [c * nw for c in s['led']]}
                    for s in states
                ]
            else:
                for fs, s in zip(final_states, states):
                    fs['theta'] += s['theta'] * nw
                    fs['phi']   += s['phi']   * nw
                    for j in range(3):
                        fs['led'][j] += s['led'][j] * nw

        return final_states

    # -------------------------
    # Behaviors
    # -------------------------
    # Each behavior has signature: (self, dt) -> list[state_dict] | None
    # state_dict: {'theta': float, 'phi': float, 'led': [r, g, b]}
    #
    # Step 2 — implement new behaviors here.

    def _sine(self, dt):
        """Theta rotates at constant velocity; phi oscillates sinusoidally per robot."""
        p = self.smoothed_params['sine']
        states = []
        for i, _ in enumerate(self.robots):
            self.phase_phi[i] = (
                self.phase_phi[i] + 2 * math.pi * p['frequency_phi'] * dt
            ) % (2 * math.pi)
            self.sine_theta_positions[i] = (
                self.sine_theta_positions[i] + p['theta_velocity'] * dt
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
            theta = (p['azimuth_offset'] + 1.0 * math.cos(angle)) % 360.0
            phi = p['tilt_angle'] + 1.0 * math.sin(angle)
            states.append({'theta': theta, 'phi': phi, 'led': [0.5, 0.5, 0.5]})
        return states

    def _quaternion(self, dt):
        """
        Drive each robot's orientation from a per-robot unit quaternion.

        Returns None if no quaternion data has been set (behavior is silently inactive).
        Feed data each frame via setQuaternion() or setAllQuaternions() before calling Update().
        """
        quats = self.behaviors['quaternion'].get('quaternions')
        if not quats or len(quats) < len(self.robots):
            return None

        states = []
        for i, _ in enumerate(self.robots):
            qx, qy, qz, qw = quats[i]
            theta, phi = self._quaternionToSpherical(qx, qy, qz, qw)
            states.append({'theta': theta, 'phi': phi, 'led': [0.0, 1.0, 1.0]})
        return states

    @staticmethod
    def _quaternionToSpherical(qx, qy, qz, qw):
        """
        Convert a unit quaternion to spherical coordinates (degrees).

        Extracts the direction of the rotated Z-axis (forward vector).
          theta — azimuth in [0, 360)
          phi   — elevation in [-90, 90]  (maps to robot phi range)
        """
        # Rotate forward vector (0, 0, 1) by the quaternion
        fx = 2.0 * (qx * qz + qy * qw)
        fy = 2.0 * (qy * qz - qx * qw)
        fz = 1.0 - 2.0 * (qx * qx + qy * qy)

        theta = math.degrees(math.atan2(fx, fz)) % 360.0
        phi   = math.degrees(math.asin(max(-1.0, min(1.0, fy))))
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
                    current[key] = cv + (tv - cv) * alpha
                else:
                    current[key] = tv  # non-numeric params pass through unchanged

    # -------------------------
    # Apply to robots
    # -------------------------

    def _applyStates(self, states, dt):
        for i, (r, s) in enumerate(zip(self.robots, states)):
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
                (self.sine_theta_positions[i] + p['theta_velocity'] * duration)
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
            theta = (p['azimuth_offset'] + 1.0 * math.cos(angle)) % 360.0
            phi = p['tilt_angle'] + 1.0 * math.sin(angle)
            states.append({'theta': theta, 'phi': phi, 'led': [0.5, 0.5, 0.5]})
        return states

    # -------------------------
    # Parameter controls
    # -------------------------

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

    def setQuaternion(self, robot_index, qx, qy, qz, qw):
        """Set the driving quaternion for one robot (for the 'quaternion' behavior)."""
        quats = self.behaviors['quaternion'].get('quaternions')
        if quats is None:
            quats = [(0.0, 0.0, 0.0, 1.0)] * len(self.robots)
            self.behaviors['quaternion']['quaternions'] = quats
        if 0 <= robot_index < len(quats):
            quats[robot_index] = (qx, qy, qz, qw)

    def setAllQuaternions(self, quaternion_list):
        """Set quaternions for all robots at once. quaternion_list: [(qx,qy,qz,qw), ...]"""
        self.behaviors['quaternion']['quaternions'] = list(quaternion_list)

    def zeroAll(self):
        for name in self.weights:
            self.weights[name] = 0.0

    # -------------------------
    # Playback controls
    # -------------------------

    def stop(self):
        """Pause motion, holding the current position."""
        self.paused       = True
        self.fade         = None
        self.faded_states = None
        self.last_states  = self._getCurrentStates()

    def resume(self):
        """Resume motion using current behavior weights."""
        self.paused = False

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
    # Output
    # -------------------------

    def GetAllStates(self):
        return [r.GetState() for r in self.robots]

    def ExposedMethod(self):
        print('ExposedMethod called!')
