#!/usr/bin/python
'''
This is the main class for running the entire game framework
'''
#import modules and classes

#import from libraries
import enum

#import from ros

class StateMachine(enum.Enum):
	S_ROBOT_ASSIST = 1
	S_USER_ACTION = 2
	S_USER_PICK_TOKEN = 3
	S_ROBOT_FEEDBACK = 4
	S_USER_PLACE = 5
	S_USER_PLACE_TOKEN_BACK = 6
	S_USER_PLACE_TOKEN_SOL = 7
	S_ROBOT_OUTCOME = 8

	CURRENT_STATE = 1

	b_robot_assist_finished = False
	b_robot_feedback_finished = False
	b_user_picked_token = False
	b_user_placed_token_back = False
	b_user_placed_token_sol = False
	b_robot_outcome_finised = False

def robot_provide_assistance(sm):
	'''
	Robot action of assistance combining speech and gesture
	:return: True when the action has been completed
	'''
	print("R_ASSISTANCE")
	sm.b_robot_assist_finished = True
	sm.CURRENT_STATE = sm.S_USER_ACTION
	return  sm.b_robot_assist_finished

def robot_provide_feedback(sm):
	'''
	The robot provides a feddback on the grasped token
	:return:
	'''
	print("R_FEEDBACK")
	sm.b_robot_feedback_finished = True
	sm.CURRENT_STATE = sm.S_USER_PLACE
	return sm.b_robot_feedback_finished

def robot_provide_outcome(sm):
	'''
	Robot provides the user with the outcome of their move
	:param sm:
	:return:
	'''
	print("R_OUTCOME")
	sm.b_robot_outcome_finished = True
	sm.CURRENT_STATE = sm.S_ROBOT_ASSIST
	return sm.b_robot_outcome_finished

def user_action(sm):
	''' Dispach user action'''
	'''We wait until the user pick a token'''
	print("U_ACTION")
	def user_pick_token(sm):
		print("U_PICK")
		sm.CURRENT_STATE = sm.S_ROBOT_FEEDBACK
		sm.user_picked_token = True
		return sm.user_picked_token
	def robot_provide_feedback(sm):
		print("R_FEEDBACK")
		sm.CURRENT_STATE = sm.S_USER_PLACE
		sm.robot_provided_feeback_finished = True
		return sm.robot_provided_feeback_finished
	''' When the user picked the token we check where they place it'''
	'''either they can place it back '''
	def user_place(sm):
		print("U_PLACE")
		def user_place_token_back(sm):
			print("U_PLACE_BACK")
			sm.CURRENT_STATE = sm.S_USER_ACTION
			sm.b_user_placed_token_back = True
			return sm.b_user_placed_token_back
		'''or they can place it in the solution row'''
		def user_place_token_sol(sm):
			print("U_PLACE_SOL")
			sm.CURRENT_STATE = sm.S_ROBOT_OUTCOME
			sm.b_user_placed_token_sol = True
			return sm.b_user_placed_token_sol
		'''return where the user placed the token'''
		#here we get the token that has been placed and trigger one or the other action
		if user_place_token_back(sm):
			sm.CURRENT_STATE = sm.S_USER_ACTION
		elif user_place_token_sol(sm):
			sm.CURRENT_STATE = sm.S_ROBOT_OUTCOME

	sm.CURRENT_STATE = sm.S_USER_ACTION
	if user_pick_token(sm):
		if robot_provide_feedback(sm):
			if user_place(sm):
				return True
		else:
			sm.CURRENT_STATE = sm.S_USER_PICK_TOKEN
	else:
		sm.CURRENT_STATE = sm.S_USER_ACTION

def num_to_func_to_str(argument):
	switcher = {
	        0: robot_provide_assistance,
	        1: robot_provide_feedback,
	        2: user_action,
		    3: robot_provide_outcome
	    }

	# get the function based on argument
	func = switcher.get(argument)

	# Execute the function
	return func()

sm = StateMachine(1)
i=0
while i<100:
	if sm.CURRENT_STATE.value == sm.S_ROBOT_ASSIST.value:
		robot_provide_assistance(sm)
	elif sm.CURRENT_STATE.value == sm.S_USER_ACTION.value:
		user_action(sm)
	elif sm.CURRENT_STATE.value == sm.S_ROBOT_OUTCOME.value:
		robot_provide_outcome()
	i += 1