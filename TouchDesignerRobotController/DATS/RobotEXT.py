class RobotEXT:
	"""
	State container for a single robot arm (theta stepper + phi servo).
	Stores motor positions/velocities and outputs to a Constant CHOP for TouchDesigner.
	"""

	def __init__(self, ownerComp):
		self.ownerComp = ownerComp

		# Motor state (spherical coordinates)
		self.theta = 0.0   # Azimuth angle, degrees [0, 360)
		self.phi   = 90.0  # Elevation angle, degrees [-135, 135]
		self.theta_velocity      = 0.0
		self.phi_velocity        = 0.0
		self.prev_theta_velocity = 0.0
		self.prev_phi_velocity   = 0.0

		# Stepper resolution
		self.pulses_per_revolution = 20000

		# Bounds
		self.min_theta, self.max_theta = 0.0, 360.0
		self.min_phi,   self.max_phi   = -135.0, 135.0

		# Velocity / acceleration limits
		self.max_theta_velocity     = 250.0  # deg/s
		self.max_phi_velocity       = 150.0  # deg/s
		self.max_theta_acceleration = 100.0  # deg/s²
		self.max_phi_acceleration   = 100.0  # deg/s²

	# -------------------------
	# Getters
	# -------------------------

	def getTheta(self):
		return self.theta

	def getPhi(self):
		return self.phi

	def GetState(self):
		return {
			'theta':          self.theta,
			'phi':            self.phi,
			'theta_velocity': self.theta_velocity,
			'pulse_per_sec':  (self.theta_velocity / 360.0) * self.pulses_per_revolution,
		}

	# -------------------------
	# Configuration
	# -------------------------

	def setPulsesPerRevolution(self, pulses):
		self.pulses_per_revolution = pulses

	def setThetaBounds(self, min_theta, max_theta):
		self.min_theta, self.max_theta = min_theta, max_theta

	def setPhiBounds(self, min_phi, max_phi):
		self.min_phi, self.max_phi = min_phi, max_phi

	def setMaxVelocity(self, max_velocity):
		self.max_theta_velocity = max_velocity
		self.max_phi_velocity   = max_velocity

	def setMaxThetaVelocity(self, max_velocity):
		self.max_theta_velocity = max_velocity

	def setMaxPhiVelocity(self, max_velocity):
		self.max_phi_velocity = max_velocity

	def setMaxAcceleration(self, max_acceleration):
		self.max_theta_acceleration = max_acceleration
		self.max_phi_acceleration   = max_acceleration

	def setMaxThetaAcceleration(self, max_acceleration):
		self.max_theta_acceleration = max_acceleration

	def setMaxPhiAcceleration(self, max_acceleration):
		self.max_phi_acceleration = max_acceleration

	# -------------------------
	# Internal helpers
	# -------------------------

	@staticmethod
	def _normalizeAngle(angle, wrap=360.0):
		return angle % wrap

	@staticmethod
	def _shortestAngleDelta(target, source, wrap=360.0):
		return ((target - source + wrap * 0.5) % wrap) - wrap * 0.5

	@staticmethod
	def _clamp(value, lo, hi):
		return max(lo, min(hi, value))

	# -------------------------
	# State setters
	# -------------------------

	def setPosition(self, theta, phi):
		self.theta = self._clamp(self._normalizeAngle(theta), self.min_theta, self.max_theta)
		self.phi   = self._clamp(phi, self.min_phi, self.max_phi)

	def SetState(self, theta, phi, ledMatrix, dt=None):
		normalized_theta = self._normalizeAngle(theta)

		if dt is not None and dt > 0.0:
			# Theta — velocity and acceleration limiting
			tv = self._clamp(
				self._shortestAngleDelta(normalized_theta, self.theta) / dt,
				-self.max_theta_velocity, self.max_theta_velocity
			)
			ta = (tv - self.prev_theta_velocity) / dt
			if abs(ta) > self.max_theta_acceleration:
				ta = self._clamp(ta, -self.max_theta_acceleration, self.max_theta_acceleration)
				tv = self.prev_theta_velocity + ta * dt
			self.theta_velocity      = tv
			self.prev_theta_velocity = tv

			# Phi — velocity and acceleration limiting
			pv = self._clamp(
				(phi - self.phi) / dt,
				-self.max_phi_velocity, self.max_phi_velocity
			)
			pa = (pv - self.prev_phi_velocity) / dt
			if abs(pa) > self.max_phi_acceleration:
				pa = self._clamp(pa, -self.max_phi_acceleration, self.max_phi_acceleration)
				pv = self.prev_phi_velocity + pa * dt
			self.phi_velocity      = pv
			self.prev_phi_velocity = pv

		self.setPosition(normalized_theta, phi)

	# -------------------------
	# CHOP output
	# -------------------------

	def PushToCHOP(self):
		"""Write theta, phi, and pulse_per_sec to the const_robot Constant CHOP."""
		chop = self.ownerComp.op('const_robot')
		if not chop:
			return
		pulse_per_sec = (self.theta_velocity / 360.0) * self.pulses_per_revolution
		chop.par.const0value = self.theta
		chop.par.const1value = self.phi
		chop.par.const2value = pulse_per_sec + 100000  # offset for serial protocol
