class RobotEXT:
	"""
	State container for a single robot arm (theta stepper + phi servo).
	theta is the single source of truth.
	theta_velocity and pulse_per_sec are direct computations of it.
	"""

	def __init__(self, ownerComp):
		self.ownerComp = ownerComp

		self.theta          = 0.0    # degrees [0, 360)
		self.phi            = 90.0   # degrees [-135, 135]
		self.theta_velocity = 0.0    # rps
		self.phi_velocity   = 0.0    # deg/s

		self.pulses_per_revolution   = 800    # must match DM860 DIP switch setting
		self.min_phi, self.max_phi   = -135.0, 135.0
		self.max_theta_velocity      =   1.5   # rps  (1.5 rps)
		self.max_phi_velocity        = 540.0   # deg/s (1.5 rps)
		self.max_theta_acceleration  =   2.3   # rps/s  — 0 = unlimited
		self.max_phi_acceleration    =   1.0   # deg/s² — 0 = unlimited

	# -------------------------
	# Getters
	# -------------------------

	def GetState(self):
		return {
			'theta':          self.theta,
			'phi':            self.phi,
			'theta_velocity': self.theta_velocity,
			'pulse_per_sec':  self.theta_velocity * self.pulses_per_revolution,
		}

	# -------------------------
	# Configuration
	# -------------------------

	def setPulsesPerRevolution(self, pulses):
		self.pulses_per_revolution = pulses

	def setPhiBounds(self, min_phi, max_phi):
		self.min_phi, self.max_phi = min_phi, max_phi

	def setMaxThetaVelocity(self, v):
		self.max_theta_velocity = v

	def setMaxPhiVelocity(self, v):
		self.max_phi_velocity = v

	def setMaxThetaAcceleration(self, a):
		self.max_theta_acceleration = a

	def setMaxPhiAcceleration(self, a):
		self.max_phi_acceleration = a

	# -------------------------
	# State update
	# -------------------------

	def SetState(self, theta, phi, ledMatrix, dt=None):
		prev_theta = self.theta
		prev_phi   = self.phi

		self.theta = theta % 360.0
		self.phi   = max(self.min_phi, min(self.max_phi, phi))

		if dt and dt > 0.0:
			delta_theta = ((self.theta - prev_theta + 180.0) % 360.0) - 180.0
			raw_tv      = max(-self.max_theta_velocity, min(self.max_theta_velocity, delta_theta / dt / 360.0))
			if self.max_theta_acceleration > 0.0:
				max_dv = self.max_theta_acceleration * dt
				raw_tv = self.theta_velocity + max(-max_dv, min(max_dv, raw_tv - self.theta_velocity))
			self.theta_velocity = raw_tv

			raw_pv = (self.phi - prev_phi) / dt
			if self.max_phi_acceleration > 0.0:
				max_dv = self.max_phi_acceleration * dt
				raw_pv = self.phi_velocity + max(-max_dv, min(max_dv, raw_pv - self.phi_velocity))
			self.phi_velocity = raw_pv
		else:
			self.theta_velocity = 0.0
			self.phi_velocity   = 0.0

	# -------------------------
	# CHOP output
	# -------------------------

	def Halt(self):
		"""Zero velocity and immediately push to CHOP. Safe to call from outside."""
		self.theta_velocity = 0.0
		self.phi_velocity   = 0.0
		self.PushToCHOP()

	def PushToCHOP(self):
		chop = self.ownerComp.op('const_robot')
		if not chop:
			return
		pulse_per_sec = -self.theta_velocity * self.pulses_per_revolution
		chop.par.const0value = self.theta
		chop.par.const1value = self.phi
		chop.par.const2value = pulse_per_sec + 100000  # offset for serial protocol
