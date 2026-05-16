def onValueChange(channel: Channel, sampleIndex: int, val: float, 
				  prev: float):
	lc = op('/project1/Robot_Controller/light_comp').ext.LightControllerEXT
	if val > 255:
		val = 255
		
	r=op('const_rgb').par.value0
	g=op('const_rgb').par.value1 
	b=op('const_rgb').par.value2
	rT = r
	gT = g
	for light in range(1,14,1):
		if g>0:
			gT = (gT + val) % 255
		rT = (rT + val) % 255
		print(rT)
		lc.setLight(light, brightness=255, r=rT, g=gT, b=b)
	return