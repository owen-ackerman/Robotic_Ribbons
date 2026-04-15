robotNumber = me.digits
def whileOn(channel: Channel, sampleIndex: int, val: float, 
            prev: float):
	valInt = int(val)
	if val <= 181:
		low_byte = valInt & 0xFF        # The remainder
		high_byte = (valInt >> 8) & 0xFF 
		op('serial_out').sendBytes(robotNumber,1, high_byte, low_byte)
		print(val)
		#print(255)
	else:
		ValStep = 100000 - valInt
		print(ValStep)
		ValStepAbs = abs(ValStep)
		low_byte = ValStepAbs & 0xFF        # The remainder
		mid_byte = (ValStepAbs >> 8) & 0xFF # The overflow
		high_byte = (ValStepAbs >> 16) & 0xFFFF
		sign = 0 if ValStep >= 0 else 1
		op('serial_out').sendBytes(robotNumber,2, high_byte, mid_byte, low_byte, sign)
		#print(254)
		print("0---------0")
	#print(valInt)
	return

