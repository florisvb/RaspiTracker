#!/usr/bin/env python
import roslib; roslib.load_manifest('pi_rotary_encoder')
import rospy

from pi_rotary_encoder.msg import ParamUpdate


import gaugette.rotary_encoder
import gaugette.switch
 
A_PIN  = 7
B_PIN  = 9
SW_PIN = 8
 

class RotaryEncoder:
	def __init__(self):
		self.encoder = gaugette.rotary_encoder.RotaryEncoder.Worker(A_PIN, B_PIN)
		self.encoder.start()
		self.switch = gaugette.switch.Switch(SW_PIN)
		self.last_switch_state = None

		rospy.init_node('rotary_encoder')
		
		self.parameters = {	'shutter_speed': 400,
					'size_threshold': 3,
					'contrast_threshold': 25,
				  }

		for param, value in self.parameters.items():
			rospy.set_param(param, value)

		self.current_key = 0

		self.publisher = rospy.Publisher('parameter_update', ParamUpdate)			
		
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.main()
			r.sleep()

	def main(self):
		param_name = self.parameters.keys()[self.current_key]
		delta = self.encoder.get_delta()
		if delta != 0:
			self.parameters[param_name] += delta
			if self.parameters[param_name] < 0:
				self.parameters[param_name] = 0
			rospy.set_param(param_name, self.parameters[param_name])				
			print param_name, self.parameters[param_name]
			self.publisher.publish(ParamUpdate(param_name, self.parameters[param_name]))

		sw_state = self.switch.get_state()
		if sw_state != self.last_switch_state:
			self.last_switch_state = sw_state
			if self.last_switch_state == 0:
				self.current_key += 1
				if self.current_key >= len(self.parameters.keys()):
					self.current_key = 0
				print self.parameters.keys()[self.current_key]

if __name__ == '__main__':
	rotaryEncoder = RotaryEncoder()
