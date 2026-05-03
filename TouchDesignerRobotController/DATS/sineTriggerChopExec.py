def onOffToOn(channel: Channel, sampleIndex: int, val: float, 
			  prev: float):

	ctrl = op('/project1/Robot_Controller/controller_comp').ext.RobotControllerEXT
	# Create a wave pattern with phase offsets between robots
	ctrl.zeroAll()
	ctrl.setWeight('stepper_speed_control', 1.0)
	ctrl.setParam('stepper_speed_control','stepper_speed',op('null_vel')[0])
	ctrl.setParam('stepper_speed_control','phi_position',op('null_phi_pos')[0])
	'''
	ctrl.setWeight('sine', 1.0)
	ctrl.setPhaseShift('sine', 0)  # 30° offset per robot
	ctrl.setThetaVelocity(op('null_vel')[0])
	ctrl.setBias('sine', bias_phi=90)  # Lift phi by 0.5 radians
	ctrl.setBias('sine', bias_theta=180)  # Lift phi by 0.5 radians
	ctrl.setFrequency('sine',frequency_phi=op('null_phi_hz')[0])
	ctrl.setParam('sine','amplitude_phi',30)
	ctrl.resume()
	ctrl.fadeTo('sine',0.1)
	'''
	return
def onOnToOff(channel: Channel, sampleIndex: int, val: float, 
			  prev: float):

	ctrl = op('/project1/Robot_Controller/controller_comp').ext.RobotControllerEXT
	print('setting frequency')
	ctrl.setFrequency('sine',frequency_phi=0)
	ctrl.setParam('sine','amplitude_phi',0)

	
	return