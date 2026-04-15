import math
import time
from multiprocessing.util import debug


class RobotControllerEXT:
    def __init__(self, ownerComp):
        print("Running init")
        self.ownerComp = ownerComp

        # Find all robots automatically
        robots = []

        for i in range(1,7):
            r = ownerComp.parent().op(f'Robot{i}')
            if r:
                robots.append(r)  

        self.robots =  robots
        # Time
        self.time = 0.0
        self.previous_time = time.time()  # Track real clock time

        # Behaviors
        self.behaviors = {
            'sine': {
                'weight': 1.0,
                'theta_velocity': 30.0,
                'frequency_theta': 1.0,
                'frequency_phi': 1.0,
                'phase_shift': 0.0,
                'bias_theta': 0.0,
                'bias_phi': 90.0,
                'amplitude_theta': 0.0,
                'amplitude_phi': 30.0
            },
            'wave': {
                'weight': 0.0,
                'frequency_theta': 1.0,
                'frequency_phi': 1.0,
                'phase_shift': 0.0,
                'bias_theta': 180.0,
                'bias_phi': 90.0,
                'amplitude_theta': 45.0,
                'amplitude_phi': 30.0
            },
            'noise': {
                'weight': 0.0,
                'frequency_theta': 3.0,
                'frequency_phi': 3.0,
                'phase_shift': 0.0,
                'bias_theta': 180.0,
                'bias_phi': 90.0,
                'amplitude_theta': 10.0,
                'amplitude_phi': 10.0
            }
        }

        # Fade system
        self.fade = None
        self.faded_states = None
        self.last_states = None
        self.sine_theta_positions = [0.0 for _ in self.robots]

    # -------------------------
    # Utility Functions
    # -------------------------

    @staticmethod
    def _normalizeAngle(angle, wrap=360.0):
        """Normalize an angle to [0, wrap)."""
        return angle % wrap

    # -------------------------
    # Easing Functions
    # -------------------------

    @staticmethod
    def _easeInOutCubic(t):
        """Smooth ease-in-out cubic easing function (0 to 1)"""
        if t < 0.5:
            return 4 * t * t * t
        else:
            t = 2 * t - 2
            return 0.5 * t * t * t + 1

    # -------------------------
    # Update Loop
    # -------------------------

    def ExposedMethod(self):
        # This method can be called externally
        print('ExposedMethod has been called !')
        pass
        
    def Update(self):
        # Calculate real delta time using clock
        current_time = time.time()
        dt = current_time - self.previous_time
        self.previous_time = current_time
        
        self.time += dt

        # Handle fades
        self._updateFade(dt)

        # Collect blended state
        final_states = None

        for name, b in self.behaviors.items():
            w = b['weight']
            if w <= 0:
                continue

            states = self._runBehavior(name, dt)

            if final_states is None:
                final_states = states
            else:
                final_states = self._blendStates(final_states, states, w)

        # Override with faded states if fading
        if self.fade and self.faded_states:
            final_states = self.faded_states

        # Apply result
        if final_states:
            self._applyStates(final_states, dt)

    # -------------------------
    # Behaviors
    # -------------------------

    def _runBehavior(self, name, dt):
        if name == 'sine':
            return self._sine(dt)

        elif name == 'wave':
            return self._wave()

        elif name == 'noise':
            return self._noise()

        return None

    def _sine(self, dt):
        states = []
        theta_velocity = self.behaviors['sine']['theta_velocity']
        freq_phi = self.behaviors['sine']['frequency_phi']
        phase_shift = self.behaviors['sine']['phase_shift']
        bias_theta = self.behaviors['sine']['bias_theta']
        bias_phi = self.behaviors['sine']['bias_phi']
        amplitude_phi = self.behaviors['sine']['amplitude_phi']
        
        for i, r in enumerate(self.robots):
            phase = i * phase_shift
            self.sine_theta_positions[i] = (self.sine_theta_positions[i] + theta_velocity * dt) % 360.0
            theta = self._normalizeAngle(self.sine_theta_positions[i] + bias_theta)
            phi = math.sin(self.time * freq_phi + phase) * amplitude_phi + bias_phi
            
            states.append({
                'theta': theta,
                'phi': phi,
                'led': [1, 1, 1]
            })

        return states

    def _wave(self):
        states = []
        freq_theta = self.behaviors['wave']['frequency_theta']
        freq_phi = self.behaviors['wave']['frequency_phi']
        phase_shift = self.behaviors['wave']['phase_shift']
        bias_theta = self.behaviors['wave']['bias_theta']
        bias_phi = self.behaviors['wave']['bias_phi']
        amplitude_theta = self.behaviors['wave']['amplitude_theta']
        amplitude_phi = self.behaviors['wave']['amplitude_phi']
        
        for i, r in enumerate(self.robots):
            phase = i * phase_shift
            theta = math.sin(self.time * freq_theta + i * 0.5 + phase) * amplitude_theta + bias_theta
            phi = math.sin(self.time * freq_phi + i * 0.5 + phase) * amplitude_phi + bias_phi

            states.append({
                'theta': self._normalizeAngle(theta),
                'phi': self._normalizeAngle(phi),
                'led': [0, 0, 1]
            })

        return states

    def _noise(self):
        states = []
        freq_theta = self.behaviors['noise']['frequency_theta']
        freq_phi = self.behaviors['noise']['frequency_phi']
        phase_shift = self.behaviors['noise']['phase_shift']
        bias_theta = self.behaviors['noise']['bias_theta']
        bias_phi = self.behaviors['noise']['bias_phi']
        amplitude_theta = self.behaviors['noise']['amplitude_theta']
        amplitude_phi = self.behaviors['noise']['amplitude_phi']
        
        for i, r in enumerate(self.robots):
            phase = i * phase_shift
            theta = math.sin(self.time * freq_theta + i + phase) * amplitude_theta + bias_theta
            phi = math.sin(self.time * freq_phi + i + phase) * amplitude_phi + bias_phi

            states.append({
                'theta': self._normalizeAngle(theta),
                'phi': self._normalizeAngle(phi),
                'led': [1, 0, 0]
            })

        return states

    def _interpolateStates(self, start_states, end_states, t):
        interpolated = []
        for s, e in zip(start_states, end_states):
            # Interpolate theta with angle wrapping
            theta_diff = self._normalizeAngle(e['theta'] - s['theta'])
            if theta_diff > 180:
                theta_diff -= 360
            theta = self._normalizeAngle(s['theta'] + theta_diff * t)
            
            # Interpolate phi with angle wrapping
            phi_diff = self._normalizeAngle(e['phi'] - s['phi'])
            if phi_diff > 180:
                phi_diff -= 360
            phi = self._normalizeAngle(s['phi'] + phi_diff * t)
            
            # Interpolate LED
            led = [
                s['led'][0] + (e['led'][0] - s['led'][0]) * t,
                s['led'][1] + (e['led'][1] - s['led'][1]) * t,
                s['led'][2] + (e['led'][2] - s['led'][2]) * t,
            ]
            interpolated.append({'theta': theta, 'phi': phi, 'led': led})
        return interpolated

    # -------------------------
    # Blending
    # -------------------------

    def _blendStates(self, A, B, weight):
        blended = []

        for a, b in zip(A, B):
            blended.append({
                'theta': a['theta'] * (1 - weight) + b['theta'] * weight,
                'phi': a['phi'] * (1 - weight) + b['phi'] * weight,
                'led': [
                    a['led'][0] * (1 - weight) + b['led'][0] * weight,
                    a['led'][1] * (1 - weight) + b['led'][1] * weight,
                    a['led'][2] * (1 - weight) + b['led'][2] * weight,
                ]
            })

        return blended

    # -------------------------
    # Apply to Robots
    # -------------------------

    def _applyStates(self, states, dt):
        for r, s in zip(self.robots, states):
            r.SetState(s['theta'], s['phi'], s['led'], dt)
            r.PushToCHOP()
        self.last_states = states

    # -------------------------
    # Fade System
    # -------------------------

    def fadeTo(self, target, duration=2.0):
        start_states = self._getCurrentStates()
        end_states = self._computeTargetStates(target, duration)
        self.fade = {
            'start_states': start_states,
            'end_states': end_states,
            'time': 0.0,
            'duration': duration,
            'target': target
        }

    def _updateFade(self, dt):
        if not self.fade:
            return

        f = self.fade
        f['time'] += dt

        t = min(f['time'] / f['duration'], 1.0)
        eased_t = self._easeInOutCubic(t)

        if t >= 1.0:
            # End fade, switch to target
            self.zeroAll()
            self.behaviors[f['target']]['weight'] = 1.0
            self.fade = None
            self.faded_states = None
        else:
            self.faded_states = self._interpolateStates(f['start_states'], f['end_states'], eased_t)

    def _getWeights(self):
        return {k: v['weight'] for k, v in self.behaviors.items()}

    def _getCurrentStates(self):
        return self.last_states if self.last_states else [{'theta': 0.0, 'phi': 90.0, 'led': [0, 0, 0]} for _ in self.robots]

    def _computeTargetStates(self, target, duration):
        if target == 'sine':
            return self._computeSineStates(duration)
        elif target == 'wave':
            return self._computeWaveStates(duration)
        elif target == 'noise':
            return self._computeNoiseStates(duration)
        return None

    def _computeSineStates(self, duration=0.0):
        states = []
        theta_velocity = self.behaviors['sine']['theta_velocity']
        freq_phi = self.behaviors['sine']['frequency_phi']
        phase_shift = self.behaviors['sine']['phase_shift']
        bias_theta = self.behaviors['sine']['bias_theta']
        bias_phi = self.behaviors['sine']['bias_phi']
        amplitude_phi = self.behaviors['sine']['amplitude_phi']
        
        for i, r in enumerate(self.robots):
            phase = i * phase_shift
            # Advance theta by duration
            theta = self._normalizeAngle((self.sine_theta_positions[i] + theta_velocity * duration) + bias_theta)
            phi = math.sin((self.time + duration) * freq_phi + phase) * amplitude_phi + bias_phi
            
            states.append({
                'theta': theta,
                'phi': phi,
                'led': [1, 1, 1]
            })

        return states

    def _computeWaveStates(self, duration=0.0):
        states = []
        freq_theta = self.behaviors['wave']['frequency_theta']
        freq_phi = self.behaviors['wave']['frequency_phi']
        phase_shift = self.behaviors['wave']['phase_shift']
        bias_theta = self.behaviors['wave']['bias_theta']
        bias_phi = self.behaviors['wave']['bias_phi']
        amplitude_theta = self.behaviors['wave']['amplitude_theta']
        amplitude_phi = self.behaviors['wave']['amplitude_phi']
        
        for i, r in enumerate(self.robots):
            phase = i * phase_shift
            theta = math.sin((self.time + duration) * freq_theta + i * 0.5 + phase) * amplitude_theta + bias_theta
            phi = math.sin((self.time + duration) * freq_phi + i * 0.5 + phase) * amplitude_phi + bias_phi

            states.append({
                'theta': self._normalizeAngle(theta),
                'phi': self._normalizeAngle(phi),
                'led': [0, 0, 1]
            })

        return states

    def _computeNoiseStates(self, duration=0.0):
        states = []
        freq_theta = self.behaviors['noise']['frequency_theta']
        freq_phi = self.behaviors['noise']['frequency_phi']
        phase_shift = self.behaviors['noise']['phase_shift']
        bias_theta = self.behaviors['noise']['bias_theta']
        bias_phi = self.behaviors['noise']['bias_phi']
        amplitude_theta = self.behaviors['noise']['amplitude_theta']
        amplitude_phi = self.behaviors['noise']['amplitude_phi']
        
        for i, r in enumerate(self.robots):
            phase = i * phase_shift
            theta = math.sin((self.time + duration) * freq_theta + i + phase) * amplitude_theta + bias_theta
            phi = math.sin((self.time + duration) * freq_phi + i + phase) * amplitude_phi + bias_phi

            states.append({
                'theta': self._normalizeAngle(theta),
                'phi': self._normalizeAngle(phi),
                'led': [1, 0, 0]
            })

        return states

    # -------------------------
    # Direct Controls
    # -------------------------

    def setWeight(self, name, weight):
        if name in self.behaviors:
            self.behaviors[name]['weight'] = weight

    def setFrequency(self, name, frequency_theta=None, frequency_phi=None):
        if name in self.behaviors:
            if frequency_theta is not None:
                self.behaviors[name]['frequency_theta'] = frequency_theta
            if frequency_phi is not None:
                self.behaviors[name]['frequency_phi'] = frequency_phi

    def setThetaVelocity(self, velocity):
        if 'sine' in self.behaviors:
            self.behaviors['sine']['theta_velocity'] = velocity

    def setPhaseShift(self, name, phase_shift):
        if name in self.behaviors:
            self.behaviors[name]['phase_shift'] = phase_shift

    def setBias(self, name, bias_theta=None, bias_phi=None):
        if name in self.behaviors:
            if bias_theta is not None:
                self.behaviors[name]['bias_theta'] = bias_theta
            if bias_phi is not None:
                self.behaviors[name]['bias_phi'] = bias_phi

    def zeroAll(self):
        for b in self.behaviors.values():
            b['weight'] = 0.0

    # -------------------------
    # Output
    # -------------------------

    def GetAllStates(self):
        return [r.GetState() for r in self.robots]