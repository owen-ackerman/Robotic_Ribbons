
def onValueChange(channel: Channel, sampleIndex: int, val: float, 
                  prev: float):
	controller = op('base1').module.controller
	controller.behaviors['sine']['frequency_phi'] = val
	return
