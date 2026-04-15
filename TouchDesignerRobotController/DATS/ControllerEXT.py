class ControllerEXT:
	"""
	Controls 6 robot instances with synchronized movements, wave patterns,
	and flexible control modes. Manages startup sequences and outputs all
	robot states to CHOPs.
	"""
	
	# Movement modes
	MODE_STARTUP = 'startup'
	MODE_SYNCHRONIZED_SINE = 'synchronized_sine'
	MODE_WAVE_PATTERN = 'wave_pattern'
	MODE_SPLIT_CONTROL = 'split_control'
	MODE_IDLE = 'idle'
	
	def __init__(self, ownerComp):
		"""
		Initialize controller with 6 robots.
		
		Args:
			ownerComp: The parent DAT component
		"""
		self.ownerComp = ownerComp
		
		# Create 6 robot instances
		self.robots = [RobotExt(i, ownerComp) for i in range(6)]
		
		# Movement state
		self.currentMode = self.MODE_IDLE
		self.time = 0.0
		self.deltaTime = 0.0
		self.startTime = 0.0
		
		# Mode-specific parameters
		self.modeParams = {}
		
		# Startup sequence state
		self.startupProgress = 0.0  # 0.0 to 1.0
	
	# -------------------------
	# Robot Access
	# -------------------------
	
	def getRobot(self, index):
		"""Get a specific robot by index (0-5)"""
		if 0 <= index < 6:
			return self.robots[index]
		return None
	
	def getAllRobots(self):
		"""Get all robot instances"""
		return self.robots
	
	# -------------------------
	# Movement Modes
	# -------------------------
	
	def startupSequence(self, duration=3.0, ledColor=[1.0, 1.0, 1.0, 1.0]):
		"""
		Execute startup sequence on all robots.
		Sweeps all robots through theta and phi with specified LED color.
		
		Args:
			duration: Duration of startup sequence in seconds
			ledColor: [R, G, B, A] for startup LEDs
		"""
		self.currentMode = self.MODE_STARTUP
		self.startTime = self.time
		self.modeParams = {
			'duration': duration,
			'ledColor': ledColor
		}
	
	def synchronizedSineWave(self, thetaAmplitude=1.0, phiAmplitude=1.0, 
	                         thetaFreq=1.0, phiFreq=1.0, ledColors=None):
		"""
		Move all 6 robots in synchronized sine wave patterns.
		
		Args:
			thetaAmplitude: Amplitude of theta oscillation (radians)
			phiAmplitude: Amplitude of phi oscillation (radians)
			thetaFreq: Frequency of theta oscillation (Hz)
			phiFreq: Frequency of phi oscillation (Hz)
			ledColors: List of 6 color tuples [R, G, B, A], one per robot
		"""
		self.currentMode = self.MODE_SYNCHRONIZED_SINE
		self.modeParams = {
			'thetaAmplitude': thetaAmplitude,
			'phiAmplitude': phiAmplitude,
			'thetaFreq': thetaFreq,
			'phiFreq': phiFreq,
			'ledColors': ledColors or [[1.0, 0.0, 0.0, 1.0] for _ in range(6)]
		}
	
	def wavePattern(self, thetaAmplitude=1.0, phiAmplitude=1.0, 
	                frequency=1.0, phaseStep=math.pi/3, ledColors=None):
		"""
		Create a wave pattern with phase offsets across the 6 robots.
		Each robot is offset by phaseStep from the previous one.
		
		Args:
			thetaAmplitude: Amplitude of theta oscillation (radians)
			phiAmplitude: Amplitude of phi oscillation (radians)
			frequency: Frequency of wave (Hz)
			phaseStep: Phase offset between adjacent robots (radians)
			ledColors: List of 6 color tuples, one per robot
		"""
		self.currentMode = self.MODE_WAVE_PATTERN
		self.modeParams = {
			'thetaAmplitude': thetaAmplitude,
			'phiAmplitude': phiAmplitude,
			'frequency': frequency,
			'phaseStep': phaseStep,
			'ledColors': ledColors or [[0.0, 1.0, 0.0, 1.0] for _ in range(6)]
		}
	
	def splitControl(self, group1Indices=[0, 1], group2Indices=[2, 3, 4, 5],
	                 group1Params=None, group2Params=None):
		"""
		Control two groups of robots independently.
		E.g., 2 robots with one movement, 4 with another.
		
		Args:
			group1Indices: List of robot indices for first group
			group2Indices: List of robot indices for second group
			group1Params: Dict with theta/phiAmplitude, freq, ledColor, etc.
			group2Params: Dict with same structure
		"""
		self.currentMode = self.MODE_SPLIT_CONTROL
		self.modeParams = {
			'group1Indices': group1Indices,
			'group2Indices': group2Indices,
			'group1Params': group1Params or {},
			'group2Params': group2Params or {}
		}
	
	def setIdle(self):
		"""Set all robots to idle (hold current position)"""
		self.currentMode = self.MODE_IDLE
	
	# -------------------------
	# Update Loop
	# -------------------------
	
	def update(self, deltaTime):
		"""
		Update all robots based on current mode.
		Should be called every frame from TouchDesigner.
		
		Args:
			deltaTime: Time since last frame in seconds
		"""
		self.deltaTime = deltaTime
		self.time += deltaTime
		
		if self.currentMode == self.MODE_STARTUP:
			self._updateStartup()
		elif self.currentMode == self.MODE_SYNCHRONIZED_SINE:
			self._updateSynchronizedSine()
		elif self.currentMode == self.MODE_WAVE_PATTERN:
			self._updateWavePattern()
		elif self.currentMode == self.MODE_SPLIT_CONTROL:
			self._updateSplitControl()
		
		# Push all robot states to CHOPs
		for robot in self.robots:
			robot.pushToCHOP()
	
	def _updateStartup(self):
		"""Update startup sequence"""
		params = self.modeParams
		duration = params.get('duration', 3.0)
		elapsed = self.time - self.startTime
		progress = min(1.0, elapsed / duration)
		
		ledColor = params.get('ledColor', [1.0, 1.0, 1.0, 1.0])
		
		# Sweep through theta and phi
		theta = progress * 2 * math.pi
		phi = progress * math.pi
		
		for robot in self.robots:
			robot.setPosition(theta, phi)
			for i in range(3):
				robot.setLED(i, *ledColor)
	
	def _updateSynchronizedSine(self):
		"""Update synchronized sine wave movement"""
		params = self.modeParams
		thetaAmp = params.get('thetaAmplitude', 1.0)
		phiAmp = params.get('phiAmplitude', 1.0)
		thetaFreq = params.get('thetaFreq', 1.0)
		phiFreq = params.get('phiFreq', 1.0)
		ledColors = params.get('ledColors', [[1.0, 0.0, 0.0, 1.0] for _ in range(6)])
		
		t = self.time
		theta = thetaAmp * math.sin(2 * math.pi * thetaFreq * t)
		phi = phiAmp * math.sin(2 * math.pi * phiFreq * t)
		
		for i, robot in enumerate(self.robots):
			robot.setPosition(theta, phi)
			color = ledColors[i] if i < len(ledColors) else [1.0, 0.0, 0.0, 1.0]
			for ledIdx in range(3):
				robot.setLED(ledIdx, *color)
	
	def _updateWavePattern(self):
		"""Update wave pattern with phase offsets"""
		params = self.modeParams
		thetaAmp = params.get('thetaAmplitude', 1.0)
		phiAmp = params.get('phiAmplitude', 1.0)
		freq = params.get('frequency', 1.0)
		phaseStep = params.get('phaseStep', math.pi / 3)
		ledColors = params.get('ledColors', [[0.0, 1.0, 0.0, 1.0] for _ in range(6)])
		
		t = self.time
		
		for i, robot in enumerate(self.robots):
			phase = i * phaseStep
			theta = thetaAmp * math.sin(2 * math.pi * freq * t + phase)
			phi = phiAmp * math.sin(2 * math.pi * freq * t + phase)
			
			robot.setPosition(theta, phi)
			color = ledColors[i] if i < len(ledColors) else [0.0, 1.0, 0.0, 1.0]
			for ledIdx in range(3):
				robot.setLED(ledIdx, *color)
	
	def _updateSplitControl(self):
		"""Update split control with two independent groups"""
		params = self.modeParams
		group1Indices = params.get('group1Indices', [0, 1])
		group2Indices = params.get('group2Indices', [2, 3, 4, 5])
		group1Params = params.get('group1Params', {})
		group2Params = params.get('group2Params', {})
		
		t = self.time
		
		# Update group 1
		for idx in group1Indices:
			robot = self.robots[idx]
			theta = group1Params.get('thetaAmplitude', 0.5) * \
					math.sin(2 * math.pi * group1Params.get('thetaFreq', 1.0) * t)
			phi = group1Params.get('phiAmplitude', 0.5) * \
					math.sin(2 * math.pi * group1Params.get('phiFreq', 1.0) * t)
			robot.setPosition(theta, phi)
			
			color = group1Params.get('ledColor', [1.0, 0.0, 0.0, 1.0])
			for ledIdx in range(3):
				robot.setLED(ledIdx, *color)
		
		# Update group 2
		for idx in group2Indices:
			robot = self.robots[idx]
			theta = group2Params.get('thetaAmplitude', 0.5) * \
					math.sin(2 * math.pi * group2Params.get('thetaFreq', 2.0) * t)
			phi = group2Params.get('phiAmplitude', 0.5) * \
					math.sin(2 * math.pi * group2Params.get('phiFreq', 2.0) * t)
			robot.setPosition(theta, phi)
			
			color = group2Params.get('ledColor', [0.0, 0.0, 1.0, 1.0])
			for ledIdx in range(3):
				robot.setLED(ledIdx, *color)
	
	# -------------------------
	# Utility Methods
	# -------------------------
	
	def getStatus(self):
		"""Return current controller status"""
		return {
			'mode': self.currentMode,
			'time': self.time,
			'robotStates': [robot.getState() for robot in self.robots]
		}