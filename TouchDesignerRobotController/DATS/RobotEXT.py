import math

class RobotEXT:
	"""
	Represents a single robot with two motors (theta, phi) and three RGB+A LEDs.
	Stores motor positions and colors, outputs to a CHOP for consumption by TouchDesigner.
	"""
	
	def __init__(self, ownerComp):
		"""
		Initialize a robot instance.
		
		Args:
			robotIndex: Integer index of this robot (0-5)
			ownerComp: The parent DAT component containing CHOPs for output
		"""

		self.ownerComp = ownerComp
		
		# Motor state (spherical coordinates)
		self.theta = self._normalizeAngle(0.0)  # Azimuth angle (degrees)
		self.phi = 90.0    # Polar angle (degrees)
		self.theta_velocity = 0.0
		self.phi_velocity = 0.0
		
		# Stepper motor parameters
		self.pulses_per_revolution = 20000  # Default value, can be set externally		
		# Bounds and limits
		self.max_theta = 360.0
		self.min_theta = 0.0
		self.max_phi = 135.0
		self.min_phi = -135.0
		self.max_theta_velocity = 250.0  # degrees/sec
		self.max_phi_velocity = 150.0  # degrees/sec
		self.max_theta_acceleration = 100.0  # degrees/sec^2
		self.max_phi_acceleration = 100.0  # degrees/sec^2
		self.prev_theta_velocity = 0.0
		self.prev_phi_velocity = 0.0		
	
	# -------------------------
	# State Getters
	# -------------------------
	
	def getTheta(self):
		"""Return current theta motor position (degrees)"""
		return self.theta
	
	def getPhi(self):
		"""Return current phi motor position (degrees)"""
		return self.phi

	
	def GetState(self):
		"""Return complete state dictionary"""
		pulse_per_sec = (self.theta_velocity / 360.0) * self.pulses_per_revolution
		return {
			'theta': self.theta,
			'phi': self.phi,
			'theta_velocity': self.theta_velocity,
			'pulse_per_sec': pulse_per_sec
		}
	
	# -------------------------
	# State Setters
	# -------------------------
	
	def _normalizeAngle(self, angle, wrap=360.0):
		"""Normalize an angle to [0, wrap)."""
		return angle % wrap

	def setPulsesPerRevolution(self, pulses):
		"""
		Set the number of pulses per revolution for the stepper motor.
		
		Args:
			pulses: Number of steps/pulses per full rotation
		"""
		self.pulses_per_revolution = pulses

	def setThetaBounds(self, min_theta, max_theta):
		"""
		Set the bounds for theta angle.
		
		Args:
			min_theta: Minimum theta angle (degrees)
			max_theta: Maximum theta angle (degrees)
		"""
		self.min_theta = min_theta
		self.max_theta = max_theta

	def setPhiBounds(self, min_phi, max_phi):
		"""
		Set the bounds for phi angle.
		
		Args:
			min_phi: Minimum phi angle (degrees)
			max_phi: Maximum phi angle (degrees)
		"""
		self.min_phi = min_phi
		self.max_phi = max_phi

	def setMaxVelocity(self, max_velocity):
		"""
		Set the maximum velocity for both theta and phi.
		
		Args:
			max_velocity: Maximum velocity in degrees/sec
		"""
		self.max_theta_velocity = max_velocity
		self.max_phi_velocity = max_velocity

	def setMaxThetaVelocity(self, max_velocity):
		"""
		Set the maximum theta velocity.
		"""
		self.max_theta_velocity = max_velocity

	def setMaxPhiVelocity(self, max_velocity):
		"""
		Set the maximum phi velocity.
		"""
		self.max_phi_velocity = max_velocity

	def setMaxAcceleration(self, max_acceleration):
		"""
		Set the maximum acceleration for both theta and phi.
		
		Args:
			max_acceleration: Maximum acceleration in degrees/sec^2
		"""
		self.max_theta_acceleration = max_acceleration
		self.max_phi_acceleration = max_acceleration

	def setMaxThetaAcceleration(self, max_acceleration):
		"""
		Set the maximum theta acceleration.
		"""
		self.max_theta_acceleration = max_acceleration

	def setMaxPhiAcceleration(self, max_acceleration):
		"""
		Set the maximum phi acceleration.
		"""
		self.max_phi_acceleration = max_acceleration

	def setPosition(self, theta, phi):
		"""
		Set motor positions.
		
		Args:
			theta: Azimuth angle in degrees
			phi: Polar angle in degrees
		"""
		normalized_theta = self._normalizeAngle(theta)
		self.theta = max(self.min_theta, min(self.max_theta, normalized_theta))
		self.phi = max(self.min_phi, min(self.max_phi, phi))
	
	def _shortestAngleDelta(self, target, source, wrap=360.0):
		"""
		Compute the smallest signed delta between two circular angles.
		Handles wrap-around correctly for 360-degree motion.
		"""
		return ((target - source + wrap * 0.5) % wrap) - wrap * 0.5
	
	def SetState(self, theta, phi, ledMatrix, dt=None):
		"""Set both motor position and LED colors"""
		normalized_theta = self._normalizeAngle(theta)
		if dt is not None and dt > 0.0:
			# Theta velocity and acceleration
			theta_velocity = self._shortestAngleDelta(normalized_theta, self.theta)/ dt
			theta_velocity = max(-self.max_theta_velocity, min(self.max_theta_velocity, theta_velocity))
			theta_accel = (theta_velocity - self.prev_theta_velocity) / dt
			if abs(theta_accel) > self.max_theta_acceleration:
				theta_accel = max(-self.max_theta_acceleration, min(self.max_theta_acceleration, theta_accel))
				theta_velocity = self.prev_theta_velocity + theta_accel * dt
			self.theta_velocity = theta_velocity
			self.prev_theta_velocity = theta_velocity

			# Phi velocity and acceleration
			phi_delta = phi - self.phi
			phi_velocity = phi_delta / dt
			phi_velocity = max(-self.max_phi_velocity, min(self.max_phi_velocity, phi_velocity))
			phi_accel = (phi_velocity - self.prev_phi_velocity) / dt
			if abs(phi_accel) > self.max_phi_acceleration:
				phi_accel = max(-self.max_phi_acceleration, min(self.max_phi_acceleration, phi_accel))
				phi_velocity = self.prev_phi_velocity + phi_accel * dt
			self.phi_velocity = phi_velocity
			self.prev_phi_velocity = phi_velocity
		self.setPosition(normalized_theta, phi)
	
	# -------------------------
	# Output to CHOP
	# -------------------------
	
	def PushToCHOP(self):
		"""
		Push robot state to output CHOP.
		Creates channels: theta, phi, pulse_per_sec
		"""
		chopName = f'const_robot'
		chop = self.ownerComp.op(chopName)
		
		if not chop:
			return
		
		# Compute pulse per second from theta velocity
		pulse_per_sec = (self.theta_velocity / 360.0) * self.pulses_per_revolution
		
		# Motor positions and pulse rate
		chop.par.const0value = self.theta
		chop.par.const1value = self.phi
		chop.par.const2value = pulse_per_sec +100000 # for serial communication.
		