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
from racecar_flexbe_states.turn_state import TurnState
from racecar_flexbe_states.drive_blind import GoBlindState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Oct 31 2019
@author: Earl
'''
class DriveCarSM(Behavior):
	'''
	It enables us to drive the car
	'''


	def __init__(self):
		super(DriveCarSM, self).__init__()
		self.name = 'Drive Car'

		# parameters of this behavior
		self.add_parameter('my_speed', -0.06)
		self.add_parameter('my_travel_dist', 100)
		self.add_parameter('my_obstacle_dist', 4000)
		self.add_parameter('my_t_speed', -0.06)
		self.add_parameter('my_turn_angle', -0.6)
		self.add_parameter('my_forward_dist', 4000)
		self.add_parameter('my_timeout', 50)
		self.add_parameter('my_proportional_turning_constant', 0.001)
		self.add_parameter('my_speed2', -0.06)
		self.add_parameter('my_angle_diff_thresh', 5)
		self.add_parameter('my_time', 15)
		self.add_parameter('my_obstacle_dist2', 2000)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:373, x:131 y:319
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:232 y:191
			OperatableStateMachine.add('Drive Forward State 1',
										GoFowardState(speed=self.my_speed, travel_dist=self.my_travel_dist, obstacle_dist=self.my_obstacle_dist, proportional_turning_constant=self.my_proportional_turning_constant, angle_diff_thresh=self.my_angle_diff_thresh),
										transitions={'failed': 'failed', 'done': 'Turn State 1'},
										autonomy={'failed': Autonomy.Off, 'done': Autonomy.High})

			# x:436 y:191
			OperatableStateMachine.add('Turn State 1',
										TurnState(t_speed=self.my_t_speed, turn_angle=self.my_turn_angle, forward_dist=self.my_forward_dist, timeout=self.my_timeout),
										transitions={'failed': 'failed', 'done': 'Drive Forward State 2'},
										autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off})

			# x:652 y:457
			OperatableStateMachine.add('Turn State 2',
										TurnState(t_speed=self.my_t_speed, turn_angle=self.my_turn_angle, forward_dist=self.my_forward_dist, timeout=self.my_timeout),
										transitions={'failed': 'failed', 'done': 'Drive Foward State 3'},
										autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off})

			# x:392 y:386
			OperatableStateMachine.add('Drive Foward State 3',
										GoFowardState(speed=self.my_speed, travel_dist=self.my_travel_dist, obstacle_dist=self.my_obstacle_dist, proportional_turning_constant=self.my_proportional_turning_constant, angle_diff_thresh=self.my_angle_diff_thresh),
										transitions={'failed': 'failed', 'done': 'finished'},
										autonomy={'failed': Autonomy.Off, 'done': Autonomy.High})

			# x:821 y:313
			OperatableStateMachine.add('Go Blind State',
										GoBlindState(speed=self.my_speed2, time=self.my_time),
										transitions={'failed': 'failed', 'done': 'Drive Forward State 2'},
										autonomy={'failed': Autonomy.Off, 'done': Autonomy.Off})

			# x:656 y:189
			OperatableStateMachine.add('Drive Forward State 2',
										GoFowardState(speed=self.my_speed2, travel_dist=self.my_travel_dist, obstacle_dist=self.my_obstacle_dist2, proportional_turning_constant=self.my_proportional_turning_constant, angle_diff_thresh=self.my_angle_diff_thresh),
										transitions={'failed': 'Go Blind State', 'done': 'Turn State 2'},
										autonomy={'failed': Autonomy.High, 'done': Autonomy.High})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
