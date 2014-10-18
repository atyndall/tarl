import serial
import codecs
import time

import numpy as np
import numpy.random
import matplotlib.pyplot as plt
import matplotlib.animation as animation

PORT = 2
BAUD = 115200

def decode_packet(packet):
	decoded_packet = {}

	for line in packet:
		parted = line.partition(" ")
		cmd = parted[0]
		val = parted[2]

		if cmd == "CPIX":
			#decoded_packet['cpix'] = int(val)
			pass
		elif cmd == "PTAT":
			#decoded_packet['ptat'] = int(val)
			pass
		elif cmd == "EEPROM":
			#decoded_packet['eeprom'] = list(bytearray(codecs.decode(val, 'hex_codec')))
			pass
		elif cmd == "IRRAW":
			#decoded_packet['ir_raw'] = [int(x) for x in val.split("\t")]
			pass
		elif cmd == "IRCLEAN":
			decoded_packet['ir_clean'] = [float(x) for x in val.split("\t")]

	return decoded_packet


if __name__ == "__main__":
	ser = serial.Serial(port=PORT, baudrate=BAUD)
	time.sleep(1)

	arr = [
		[20]*16,
		[25]*16,
		[25]*16,
		[30]*16,
	]

	plt.ion()
	plt.imshow(arr, vmin=15, vmax=50)
	plt.colorbar()
	plt.pause(0.0001)
	
	while True:
		line = ser.readline().decode("ascii", "ignore").strip()
		msg = []

		# Capture a whole packet
		while line != "START":
			line = ser.readline().decode("ascii", "ignore").strip()

		while line != "STOP":
			msg.append(line)
			line = ser.readline().decode("ascii", "ignore").strip()

		dpct = decode_packet(msg)
		if 'ir_clean' in dpct:
			pct = dpct['ir_clean']

			arr = [
				pct[0:16],
				pct[16:32],
				pct[32:48],
				pct[48:64]
			]

			plt.clf()
			plt.imshow(arr, vmin=15, vmax=50)
			plt.colorbar()
			plt.pause(0.0001)  