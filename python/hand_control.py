import serial
from serial.tools import list_ports
import time
import numpy as np
import struct
import threading
from PPP_stuffing import *
from abh_api_core import *


class AbilityHandControl:
	def __init__(self, sleep_time=0.01, sleep_fn=None):
		self.port = None
		self.baudrate = 460800
		self.hand_address = 0x50
		self.width = 500
		self.reply_mode_header = 0x10
		self.read_only_mode_header = 0xA0
		self.sleep_time = sleep_time
		self.serial: serial.Serial = None

		self.sleep_fn = sleep_fn or (lambda x: time.sleep(x))

		self.open_interrupt_flag = False
		self.close_interrupt_flag = False

		self.finger_pos_min = [5.0, 5.0, 5.0, 5.0, 5.0, 5.0]
		## Cap at 90 for fingers, 50 for thumb to avoid collision
		self.finger_pos_max = [90, 90, 90, 90, 50, -50]

		self.fingers_opened = False
		self.fingers_closed = False

		self.lock = threading.Lock()

		if not self.connect():
			print("Failed to connect to hand")
			return

		# Upsample the thumb rotator
		msg = self.create_misc_msg(0xC2)
		self.serial.write(msg)

		# read current positions
		msg = self.read_only_mode()
		self.serial.write(PPP_stuff(bytearray(msg)))
		pos, _, _ = self.read_data()
		print(f'Current positions: {pos}')

	def side_grasp(self):
		self.close_fingers([5], non_blocking=True)
		self.close_fingers_threaded([0, 1, 2, 3, 4])

	def open_close(self, idx=None):
		while True:
			if idx is None:
				self.open_hand()
				self.close_hand()
			else:
				self.open_fingers(idx)
				self.close_fingers(idx)

	def open_fingers(self, finger_indices, limits_min=None, non_blocking=True):
		# open hand to max position
		limits_min = limits_min or self.finger_pos_min

		# read current positions
		msg = self.read_only_mode()
		self.serial.write(PPP_stuff(bytearray(msg)))
		pos, _, _ = self.read_data()

		new_pos = pos.copy()

		self.serial.reset_input_buffer()

		self.fingers_opened = False
		self.open_interrupt_flag = False

		if self.lock.locked():
			print('data locked')
			return

		self.lock.acquire()

		while True and not self.open_interrupt_flag:
			for i in finger_indices:
				if i == 5:
					new_pos[i] = new_pos[i] + 0.5
					new_pos[i] = min(new_pos[i], limits_min[i])
				else:
					new_pos[i] = new_pos[i] - 0.5
					new_pos[i] = max(new_pos[i], limits_min[i])

			msg = self.generateTX(new_pos)
			self.serial.write(PPP_stuff(bytearray(msg)))
			self.sleep_fn(self.sleep_time)
			self.serial.reset_input_buffer()

			# break if required fingers are open
			if all(new_pos[i] == limits_min[i] for i in finger_indices):
				self.fingers_opened = True

			# break when interrupted
			if self.fingers_opened and (non_blocking or self.open_interrupt_flag):
				break

		self.lock.release()

	def open_fingers_threaded(self, finger_indices, limits_min=None):
		while self.lock.locked():
			print('data locked')
			self.sleep_fn(self.sleep_time)
		
		thread = threading.Thread(target=self.open_fingers, args=(finger_indices, limits_min, False))
		
		thread.daemon = True
		self.fingers_opened = False

		try:
			thread.start()
			while thread.is_alive() and not self.fingers_opened:
				self.sleep_fn(self.sleep_time)
		except KeyboardInterrupt:
			self.open_interrupt_flag = True
			print('KeyboardInterrupt in open_fingers_threaded')
			thread.join()

	def close_fingers_threaded(self, finger_indices, limits_max=None):
		
		while self.lock.locked():
			print('data locked')
			self.sleep_fn(self.sleep_time)

		thread = threading.Thread(target=self.close_fingers, args=(finger_indices, limits_max, False))
		
		thread.daemon = True
		self.fingers_closed = False

		try:
			thread.start()
			while thread.is_alive() and not self.fingers_closed:
				self.sleep_fn(self.sleep_time)
		except KeyboardInterrupt:
			self.close_interrupt_flag = True
			print('KeyboardInterrupt in close_fingers_threaded')
			thread.join()

	def close_fingers(self, finger_indices, limits_max=None, non_blocking=True):
		# close hand to min position
		limits_max = limits_max or self.finger_pos_max

		# read current positions
		msg = self.read_only_mode()
		self.serial.write(PPP_stuff(bytearray(msg)))
		pos, _, _ = self.read_data()

		new_pos = pos.copy()

		self.serial.reset_input_buffer()

		self.fingers_closed = False
		self.close_interrupt_flag = False

		if self.lock.locked():
			print('data locked')
			return

		self.lock.acquire()

		while True and not self.close_interrupt_flag:
			for i in finger_indices:
				if i == 5:
					new_pos[i] = new_pos[i] - 0.5
					new_pos[i] = max(new_pos[i], limits_max[i])
				else:
					new_pos[i] = new_pos[i] + 0.5
					new_pos[i] = min(new_pos[i], limits_max[i])

			msg = self.generateTX(new_pos)
			self.serial.write(PPP_stuff(bytearray(msg)))
			self.sleep_fn(self.sleep_time)

			# pos, _, _ = self.read_data()

			self.serial.reset_input_buffer()

			if all(new_pos[i] == limits_max[i] for i in finger_indices):
				self.fingers_closed = True

			# break when interrupted
			if self.fingers_closed and (non_blocking or self.close_interrupt_flag):
				break

		self.lock.release()

	def open_hand(self):
		# open hand to max position
		self.open_fingers([0, 1, 2, 3, 4, 5])

	def close_hand(self):
		# close hand to min position
		# self.close_fingers([0, 1, 2, 3, 4, 5])
		self.close_fingers_threaded([0, 1, 2, 3, 4, 5])

	def __del__(self):
		if self.serial is not None:
			try:
				print("Closing serial port")
				# TODO: revert this
				# self.open_hand()
				self.serial.close()
			except Exception as e:
				print(e)
				self.open_hand()
				self.serial.close()

	def connect(self) -> bool:
		com_ports_list = list_ports.comports()
		for port in com_ports_list:
			if "TTL" in port.description:
				self.port = port.device
				print("Found port: ", self.port)
				break
		if self.port is None:
			print("No port found")
			return False

		try:
			self.serial = serial.Serial(self.port, self.baudrate, timeout=0)
			print("Serial port opened")
			return True
		except:
			print("Serial port open failed")
			return False

	## Send Miscellanous Command to Ability Hand
	def create_misc_msg(self, cmd):
		barr = []
		barr.append((struct.pack("<B", 0x50))[0])
		# device ID
		barr.append((struct.pack("<B", cmd))[0])
		# command!
		sum = 0
		for b in barr:
			sum = sum + b
		chksum = (-sum) & 0xFF
		barr.append(chksum)
		return barr

	## Generate Message to send to hand from array of positions (floating point)
	def generateTX(self, positions):
		txBuf = []

		## Address in byte 0
		txBuf.append((struct.pack("<B", self.hand_address))[0])

		## Format Header in byte 1
		txBuf.append((struct.pack("<B", self.reply_mode_header))[0])

		## Position data for all 6 fingers, scaled to fixed point representation
		for i in range(0, 6):
			posFixed = int(positions[i] * 32767 / 150)
			txBuf.append((struct.pack("<B", (posFixed & 0xFF)))[0])
			txBuf.append((struct.pack("<B", (posFixed >> 8) & 0xFF))[0])

		## calculate checksum
		cksum = 0
		for b in txBuf:
			cksum = cksum + b
		cksum = (-cksum) & 0xFF
		txBuf.append((struct.pack("<B", cksum))[0])

		return txBuf

	def read_only_mode(self):
		txBuf = []

		## Address in byte 0
		txBuf.append((struct.pack("<B", self.hand_address))[0])

		## Format Header in byte 1
		txBuf.append((struct.pack("<B", self.read_only_mode_header))[0])

		## calculate checksum
		cksum = 0
		for b in txBuf:
			cksum = cksum + b
		cksum = (-cksum) & 0xFF
		txBuf.append((struct.pack("<B", cksum))[0])

		return txBuf

	def read_data(self, size=512):
		bytebuffer = bytes([])
		stuff_buffer = np.array([])

		nb = bytes([])
		while len(nb) == 0:
			nb = self.serial.read(size)  # gigantic read size with nonblocking

		bytebuffer = bytebuffer + nb

		if len(bytebuffer) != 0:  # redundant, but fine to keep
			npbytes = np.frombuffer(bytebuffer, np.uint8)
			for b in npbytes:
				payload, stuff_buffer = unstuff_PPP_stream(b, stuff_buffer)
				if len(payload) != 0:
					rPos, rI, rV, rFSR = parse_hand_data(payload)
					if (rPos.size + rI.size + rV.size + rFSR.size) != 0:
						posRead = rPos.copy()
						touchRead = rFSR.copy()
						currentRead = rI.copy()

						return posRead, currentRead, touchRead

		return None, None, None


if __name__ == "__main__":
	hand_control = AbilityHandControl()

	# while True:
	# 	hand_control.interrupt_flag = True
	# 	hand_control.open_fingers_threaded([0, 1, 2, 3, 4, 5])
	# 	time.sleep(1)
	# 	hand_control.interrupt_flag = True
	# 	hand_control.close_fingers_threaded([0, 1, 2, 3, 4, 5])
	# 	time.sleep(1)

	while True:
		hand_control.close_interrupt_flag = True
		hand_control.open_interrupt_flag = False
		hand_control.open_fingers_threaded([0, 1, 2, 3])
		# time.sleep(1)
		hand_control.open_interrupt_flag = True
		hand_control.close_interrupt_flag = False
		hand_control.close_fingers_threaded([0, 1, 2, 3])
		# time.sleep(1)
		hand_control.close_interrupt_flag = True
		hand_control.open_interrupt_flag = False
		hand_control.close_fingers_threaded([5])
		# # time.sleep(2)
		hand_control.close_interrupt_flag = True
		hand_control.open_interrupt_flag = False
		hand_control.open_fingers_threaded([5])
		# time.sleep(2)
		hand_control.open_interrupt_flag = True
		hand_control.close_interrupt_flag = False