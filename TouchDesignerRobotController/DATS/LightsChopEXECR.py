def onValueChange(channel: Channel, sampleIndex: int, val: float, 
				  prev: float):
	lc = op('light_comp').ext.LightControllerEXT
	if val > 255:
		val = 255
	r=op('const_rgb').par.value0
	g=op('const_rgb').par.value1 
	b=op('const_rgb').par.value2
	for light in range(2,14,2):
		lc.setLight(light, brightness=val, r=r, g=g, b=b)
	return