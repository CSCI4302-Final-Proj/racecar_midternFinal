#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from racecar_flexbe_states.drive_forward import GoFowardState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Oct 31 2019
@author: Me
'''
class DriveForwardRacecarSM(Behavior):
	'''
	Enable us to drive forward
	'''


	def __init__(self):
		super(DriveForwardRacecarSM, self).__init__()
		self.name = 'Drive Forward Racecar'

		# parameters of this behavior
		self.add_parameter('my_speed', 0.4)
		self.add_parameter('my_travel_dist', 4)
		self.add_parameter('my_obstacle_dist', 1)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:170 y:158
			OperatableStateMachine.add('Drive Forward State',
										GoFowardState(speed=self.my_speed, travel_dist=self.my_travel_dist, obstacle_dist=self.my_obstacle_dist),
										transitions={'failed': 'failed', 'done': 'finished'},
										autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
