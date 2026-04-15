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
		return {
			'theta': self.theta,
			'phi': self.phi,
			'theta_velocity': self.theta_velocity
		}
	
	# -------------------------
	# State Setters
	# -------------------------
	
	def _normalizeAngle(self, angle, wrap=360.0):
		"""Normalize an angle to [0, wrap)."""
		return angle % wrap

	def setPosition(self, theta, phi):
		"""
		Set motor positions.
		
		Args:
			theta: Azimuth angle in degrees
			phi: Polar angle in degrees
		"""
		self.theta = self._normalizeAngle(theta)
		self.phi = self._normalizeAngle(phi)
	
	
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
			self.theta_velocity = self._shortestAngleDelta(normalized_theta, self.theta) / dt
		self.setPosition(normalized_theta, phi)
	
	# -------------------------
	# Output to CHOP
	# -------------------------
	
	def PushToCHOP(self):
		"""
		Push robot state to output CHOP.
		Creates channels: theta, phi, theta_velocity
		"""
		chopName = f'const_robot'
		chop = self.ownerComp.op(chopName)
		
		if not chop:
			return
		
		# Motor positions and velocity
		chop.par.const0value = self.theta
		chop.par.const1value = self.phi
		chop.par.const2value = self.theta_velocity
		