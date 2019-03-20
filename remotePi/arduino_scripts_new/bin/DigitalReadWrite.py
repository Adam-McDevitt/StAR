from time import sleep
from UUGear import *

UUGearDevice.setShowLogs(0)

device = UUGearDevice('UUGear-Arduino-1010-2838')
device1= UUGearDevice('UUGear-Arduino-9113-1960')
if device.isValid() and device1.isValid():
	device.setPinModeAsOutput(13)
	device1.setPinModeAsOutput(13)
	for i in range(5):
		device.setPinHigh(13)
		device1.setPinHigh(13)
		sleep(0.2)
		device.setPinLow(13)
		device1.setPinLow(13)
		sleep(0.2)
	
	device.setPinModeAsInput(9)
	print 'Pin 9 status= ', device.getPinStatus(9)	
	device.detach()
	device.stopDaemon()
	device1.setPinModeAsInput(9)
	print 'Pin 9 status device 1= ', device1.getPinStatus(9)
	device1.detach()
	device1.stopDaemon()
else:
	print 'UUGear device is not correctly initialized.'
