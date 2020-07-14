#!/usr/bin/python
'''
This is the main class for running the entire game framework
'''
#import modules and classes

#import from libraries
import enum
import random

#import from ros
import rospy
from board_state.msg import TokenMsg
from board_state.msg import BoardMsg

user_picked = (0, 0)
user_placed = (0, 0)


class Game(object):
	def __init__(self):
		rospy.init_node('big_hero', anonymous=True)
		# subscriber for getting info from the board
		rospy.Subscriber("/detected_move", TokenMsg, self.get_move_event_callback)
		rospy.Subscriber("/board_status", BoardMsg, self.get_board_event_callback)
		# get the objective of the exercise
		self.board = []
		self.objective = rospy.get_param("/objective")
		self.solution = self.set_objective(5)
		self.n_attempt_per_token = 1
		self.n_max_attempt = 5#n_max_attempt
		self.n_mistakes = 0
		self.n_solution = 10#n_solution
		self.n_correct_move = 0
		self.detected_token = []
		self.picked = False
		self.placed = False
		self.moved_back = False

	def get_board_event_callback(self, msg):
		'''callback from the topic board_status to get the status of the board'''
		self.board = msg.data

	def get_board_event(self):
		'''This method returns what is listened by the subscriber'''
		return self.board

	def get_move_event_callback(self, msg):
		'''callback from the topic detected_move to get the detected move if so'''
		self.detected_token = msg.detected_token
		self.picked = msg.picked
		self.placed = msg.placed
		self.moved_back = msg.moved_back

	def get_move_event(self):
		'''This method just returns what is listened by the subscriber'''
		return self.detected_token, self.picked, self.placed, self.moved_back

	def set_objective(self, n_token):
		'''The method return a list with the tokens ordered based on the solution of the exercise'''
		rospy.sleep(0.1)
		board_ = self.board[:]
		#remove the empty cells from the board
		board_filtered = list(filter(lambda x: x!="0", board_))
		if self.objective == "ascending":
			self.solution = sorted(board_filtered)[:n_token]
		elif self.objective == "descending":
			self.solution = sorted(board_filtered, reverse=False)[:n_token]
		else:
			assert("Game is not defined contact the developer for integrating it")
		return self.solution

class StateMachine(enum.Enum):

	def __init__(self, e):
		self.Game = Game()

	#initialise the node and the subscriber
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



	def robot_provide_assistance(self):
		'''
		Robot action of assistance combining speech and gesture
		:return: True when the action has been completed
		'''
		print("R_ASSISTANCE")
		rospy.sleep(2.0)
		self.b_robot_assist_finished = True
		self.CURRENT_STATE = self.S_USER_ACTION
		return  self.b_robot_assist_finished

	def robot_provide_feedback(self):
		'''
		The robot provides a feddback on the grasped token
		:return:
		'''
		print("R_FEEDBACK")
		rospy.sleep(2.0)
		self.b_robot_feedback_finished = True
		self.CURRENT_STATE = self.S_USER_PLACE
		return self.b_robot_feedback_finished

	def robot_provide_outcome(self):
		'''
		Robot provides the user with the outcome of their move
		:param self:
		:return:
		'''
		print("R_OUTCOME")
		rospy.sleep(2.0)
		outcome = 0
		#get current move and check if it is the one expeceted in the solution list
		if  self.Game.detected_token[0] == self.Game.solution[self.Game.n_correct_move]  \
			and self.Game.detected_token[2] == str(self.Game.solution.index(self.Game.detected_token[0])+1):
			outcome = 1
			self.Game.n_correct_move += 1
			print("correct_solution")
		elif self.Game.detected_token[0] == []:
			outcome = 0
			print("timeout")
			self.Game.n_mistakes += 1
			self.Game.n_attempt_per_token += 1
		elif self.Game.detected_token[0] != self.Game.solution[self.Game.n_correct_move]  \
			or self.Game.detected_token[2] != self.Game.solution.index(self.Game.detected_token[0])+1:
			outcome = -1
			print("wrong_solution")
			self.Game.n_mistakes += 1
			self.Game.n_attempt_per_token += 1

		self.b_robot_outcome_finished = True
		self.CURRENT_STATE = self.S_ROBOT_ASSIST
		return self.b_robot_outcome_finished, outcome

	def user_action(self):
		''' Dispach user action'''
		'''We wait until the user pick a token'''
		print("U_ACTION")
		def user_pick_token(sm, game):
			print("U_PICK")
			sm.CURRENT_STATE = sm.S_ROBOT_FEEDBACK
			detected_token, picked, _, _ = game.get_move_event()
			while(not picked):
				detected_token, picked, _, _ = game.get_move_event()
			sm.user_picked_token = True
			user_picked = detected_token
			return sm.user_picked_token
		def robot_provide_feedback(sm):
			print("R_FEEDBACK")
			rospy.sleep(2.0)
			sm.CURRENT_STATE = sm.S_USER_PLACE
			sm.robot_provided_feeback_finished = True
			return sm.robot_provided_feeback_finished
		''' When the user picked the token we check where they place it'''
		'''either they can place it back '''
		def user_place(sm, game):
			print("U_PLACE")
			#check where the user will place the token
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

			detected_token, picked, placed, moved_back = game.get_move_event()
			while (not placed):
				detected_token, _, placed, moved_back = game.get_move_event()
			if (placed and moved_back):
				user_place_token_back(sm)
				self.CURRENT_STATE = sm.S_USER_ACTION
			elif (placed and not moved_back):
				user_place_token_sol(sm)
				self.CURRENT_STATE = sm.S_ROBOT_OUTCOME
			else:
				assert "Unexpected state"

		self.CURRENT_STATE = self.S_USER_ACTION
		if user_pick_token(self, self.Game):
			if robot_provide_feedback(self):
				if user_place(self, self.Game):
					return True
			else:
				self.CURRENT_STATE = self.S_USER_PICK_TOKEN
		else:
			self.CURRENT_STATE = self.S_USER_ACTION

	def num_to_func_to_str(self, argument):
		switcher = {
		        0: self.robot_provide_assistance,
		        1: self.robot_provide_feedback,
		        2: self.user_action,
			    3: self.robot_provide_outcome
		    }

		# get the function based on argument
		func = switcher.get(argument)

		# Execute the function
		return func()




def main():

	n_solution = 5
	n_max_attempt = 5
	initial_state = 1
	game = Game()
	iter = 0
	rospy.sleep(0.1)
	current_board = game.get_board_event()
	#game.solution = game.set_objective(n_solution)
	sm = StateMachine(1)

	while game.n_correct_move<game.n_solution:
		if sm.CURRENT_STATE.value == sm.S_ROBOT_ASSIST.value:
			sm.robot_provide_assistance()
		elif sm.CURRENT_STATE.value == sm.S_USER_ACTION.value:
			sm.user_action()
		elif sm.CURRENT_STATE.value == sm.S_ROBOT_OUTCOME.value:
			sm.robot_provide_outcome()

			print("game_state:", game.n_correct_move, "n_attempt_token:", game.n_attempt_per_token)
			game.n_attempt_per_token = 1


	print("n_token ", game.n_correct_move, " n_total_attempt ", game.n_mistakes)
	iter += 1



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass