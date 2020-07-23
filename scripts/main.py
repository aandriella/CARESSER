#!/usr/bin/python
'''
This is the main class for running the entire game framework
'''
#import modules and classes
from log import Log
from cognitive_game import Game
#import from libraries
import enum
import random
import time
import os

#import from ros
import rospy

class StateMachine(enum.Enum):

	def __init__(self, e):
		self.b_robot_assist_finished = False
		self.b_robot_feedback_finished = False
		self.b_user_picked_token = False
		self.b_user_placed_token_back = False
		self.b_user_placed_token_sol = False
		self.b_robot_outcome_finised = False
		self.b_robot_moved_token_back = False
		self.b_user_moved_token_back = False
		self.b_robot_moved_correct_token = False
		self.b_user_reached_max_attempt = False
		self.b_user_reached_timeout = False
		self.b_robot_reengaged_user = False
	'''States of the State machine'''
	S_ROBOT_ASSIST = 1
	S_USER_ACTION = 2
	S_USER_PICK_TOKEN = 3
	S_ROBOT_FEEDBACK = 4
	S_USER_PLACE = 5
	S_USER_PLACE_TOKEN_BACK = 6
	S_USER_PLACE_TOKEN_SOL = 7
	S_USER_MOVE_TOKEN_BACK = 8
	S_ROBOT_MOVE_TOKEN_BACK = 9
	S_ROBOT_MOVE_CORRECT_TOKEN = 10
	S_ROBOT_OUTCOME = 11
	S_USER_TIMEOUT = 12

	CURRENT_STATE = 1

	def robot_provide_assistance(self, game):
		'''
		Robot action of assistance combining speech and gesture
		:return: True when the action has been completed
		'''
		if self.b_user_reached_timeout==True:
			print("R_REENGAGE")
			self.robot_reengage_user()
		else:
			print("R_ASSISTANCE")
			game.robot_assistance = random.randint(0,4)
			rospy.sleep(1.0)
			self.b_robot_assist_finished = True
			self.CURRENT_STATE = self.S_USER_ACTION
		return  self.b_robot_assist_finished

	def robot_provide_feedback(self):
		'''
		The robot provides a feddback on the grasped token
		:return:
		'''
		print("R_FEEDBACK")
		self.b_robot_feedback_finished = True
		self.CURRENT_STATE = self.S_USER_PLACE
		return self.b_robot_feedback_finished

	def robot_reengage_user(self):
		'''As the timeout occurred the robot reengages the user'''
		print("RE_ENGAGE USER")
		self.CURRENT_STATE = self.S_USER_ACTION
		self.b_robot_reengaged_user = True

	def robot_provide_outcome(self, game):
		'''
		Robot provides the user with the outcome of their move
		:param self:
		:return:
		'''
		print("R_OUTCOME")
		if game.moved_back:
			print("token has been moved back, DO nothing")
			game.outcome = ""
			self.CURRENT_STATE = self.S_ROBOT_ASSIST
			# check if the user reached his max number of attempts
			if game.n_attempt_per_token >= game.n_max_attempt_per_token:
				print("Max attempt reached")
				self.S_ROBOT_MOVE_CORRECT_TOKEN = True
				self.b_user_reached_max_attempt = True
				self.robot_move_correct_token(game)

		#get current move and check if it is the one expeceted in the solution list
		elif game.detected_token[0] == game.solution[game.n_correct_move]  \
			and game.detected_token[2] == str(game.solution.index(game.detected_token[0])+1):
			#get the values from attempt and mistakes before resetting them
			game.outcome = 1
			print("correct_solution ", game.get_n_correct_move())
			self.CURRENT_STATE = self.S_ROBOT_ASSIST
		elif game.detected_token[0] == []:
			game.outcome = 0
			print("timeout")
			attempt = game.n_attempt_per_token
			game.n_mistakes += 1
			game.n_attempt_per_token += 1
			game.set_attempt_per_token(game.n_attempt_per_token)
			game.set_n_mistakes(game.n_mistakes)
			attempt = game.n_attempt_per_token
			self.CURRENT_STATE = self.S_ROBOT_ASSIST

			# check if the user reached his max number of attempts
			if game.n_attempt_per_token >= game.n_max_attempt:
				self.S_ROBOT_MOVE_CORRECT_TOKEN = True
				self.b_user_reached_max_attempt = True
				self.robot_move_correct_token()

		elif game.detected_token[0] != game.solution[game.n_correct_move]  \
			or game.detected_token[2] != game.solution.index(game.detected_token[0])+1:
			game.outcome = -1
			print("wrong_solution")
			game.n_mistakes += 1
			game.n_attempt_per_token += 1
			game.set_n_attempt_per_token(game.n_attempt_per_token)
			game.set_n_mistakes(game.n_mistakes)
			attempt = game.n_attempt_per_token
			self.CURRENT_STATE = self.S_ROBOT_MOVE_TOKEN_BACK
			self.user_move_back(game)

			#check if the user reached his max number of attempts
			if game.n_attempt_per_token>=game.n_max_attempt_per_token:
				print("Max attempt reached")
				self.S_ROBOT_MOVE_CORRECT_TOKEN = True
				self.b_user_reached_max_attempt = True
				self.robot_move_correct_token(game)

		self.b_robot_outcome_finished = True
		return self.b_robot_outcome_finished

	def robot_move_back(self):
		#user moved the token in an incorrect location
		#robot moved it back
		print("Robot moved back the token")
		self.CURRENT_STATE = self.S_ROBOT_ASSIST
		self.b_robot_moved_token_back = True
		return self.b_robot_moved_token_back

	def robot_move_correct_token(self, game):
		print("Robot moves the correct token as the user reached the max number of attempts")
		#get the current solution
		token = game.solution[game.n_correct_move]
		_from = game.initial_board.index(token)
		_to = game.solution.index(token)+1
		press_key = raw_input("Please move the token {} at loc {}".format(token, _to))
		game.set_n_correct_move(game.get_n_correct_move()+1)
		game.set_n_attempt_per_token(1)

	def user_move_back(self, game):
		#user moved the token in an incorrect location
		#robot moved it back
		print("User moved back the token")
		#get the initial location of the placed token and move back there
		token, _to, _from = game.detected_token
		while(game.detected_token!=[token, _from, _to]):
			pass
		self.CURRENT_STATE = self.S_ROBOT_ASSIST
		self.b_user_moved_token_back = True
		return self.b_user_moved_token_back

	def user_action(self, game):
		'''user action state has been divided in three sub methods:
		 1. user pick a token
		 1.a check for timeout
		    1.b timeout -> provide reengagement
		    go back to 1
		 2. robot provides feedback
		 3. user places a token
		 3.a user places a token back
		 3.b user places a token in a goal location
		  '''
		print("U_ACTION")

		def check_move_timeout(game):
			'''we need to check if the computed time (react+elapsed) is smaller than timeout
			if not, we need to trigger different actions:
			1. if the user has not picked a token (re-engage)
			2. if the user has picked a token, ask her to move it back
			'''
			current_time = time.time()
			elapsed_time = current_time-game.react_time_per_token_spec_t0
			if elapsed_time<game.timeout:
				return True
			else:
				return False

		def user_pick_token(sm, game):
			game.react_time_per_token_spec_t0 = time.time()
			print("U_PICK")
			sm.CURRENT_STATE = sm.S_ROBOT_FEEDBACK
			detected_token, picked, _, _ = game.get_move_event()
			while(not picked and (detected_token==[])):
				if check_move_timeout(game):
					detected_token, picked, _, _ = game.get_move_event()
				else:
					sm.b_user_reached_timeout = True
					game.react_time_per_token_spec_t0 = time.time()
					return False

			game.elapsed_time_per_token_spec_t0 = time.time()
			game.react_time_per_token_spec_t1 = time.time()-game.react_time_per_token_spec_t0
			game.react_time_per_token_gen_t1 += game.react_time_per_token_spec_t1
			sm.b_user_picked_token = True
			return sm.b_user_picked_token

		def robot_provide_feedback(sm, game):
			print("R_FEEDBACK")
			#if the
			game.n_sociable_per_token += 1
			game.n_tot_sociable += 1
			# if sm.b_user_picked_token and not game.detected_token[0] == game.solution[game.n_correct_move]:
			# 	game.n_attempt_per_token += 1

			sm.CURRENT_STATE = sm.S_USER_PLACE
			sm.robot_provided_feeback_finished = True
			return sm.robot_provided_feeback_finished
		''' When the user picked the token we check where they place it'''
		'''either they can place it back '''
		def user_place(sm, game):
			'''user place has been divided into:
			   1. user places a token back
			   2. user places a token in a solution row
			'''
			print("U_PLACE")
			#check where the user will place the token
			def user_place_token_back(sm):
				print("U_PLACE_BACK")
				sm.CURRENT_STATE = sm.S_ROBOT_OUTCOME
				sm.b_user_placed_token_back = True
				game.elapsed_time_per_token_spec_t1 = time.time()-game.elapsed_time_per_token_spec_t0
				game.elapsed_time_per_token_gen_t1 += game.elapsed_time_per_token_spec_t1
				game.n_mistakes += 1
				game.n_attempt_per_token += 1
				game.set_n_attempt_per_token(game.n_attempt_per_token)
				game.set_n_mistakes(game.n_mistakes)
				return sm.b_user_placed_token_back
			'''or they can place it in the solution row'''
			def user_place_token_sol(sm):
				print("U_PLACE_SOL")
				sm.CURRENT_STATE = sm.S_ROBOT_OUTCOME
				sm.b_user_placed_token_sol = True
				game.elapsed_time_per_token_spec_t1 = time.time() - game.elapsed_time_per_token_spec_t0
				game.elapsed_time_per_token_gen_t1 += game.elapsed_time_per_token_spec_t1
				return sm.b_user_placed_token_sol
			'''return where the user placed the token'''

			#here we check whether a token has been picked, and where it has been placed
			detected_token, picked, placed, moved_back = game.get_move_event()
			while (not placed):
				detected_token, _, placed, moved_back = game.get_move_event()
			if (placed and moved_back):
				self.CURRENT_STATE = sm.S_ROBOT_OUTCOME
				return user_place_token_back(sm)
			elif (placed and not moved_back):
				self.CURRENT_STATE = sm.S_ROBOT_OUTCOME
				return user_place_token_sol(sm)
			else:
				assert "Unexpected state"

		self.CURRENT_STATE = self.S_USER_ACTION
		#if the user picks a token and SOCIABLE is active
		if user_pick_token(self, game):
			if game.with_SOCIABLE:
				if robot_provide_feedback(self, game):
					if user_place(self, game):
						return True
					else:
						print("Something went wrong with user place in SOCIABLE")
				else:
					print("Something went wrong with robot feedback")
			else:
				if user_place(self, game):
					return True
				else:
					print("Something went wrong with user place")
		else:
			self.CURRENT_STATE = self.S_ROBOT_ASSIST

	def num_to_func_to_str(self, argument):
		switcher = {
		        0: self.robot_provide_assistance,
				1: self.robot_reengage_user,
		        2: self.robot_provide_feedback,
		        3: self.user_action,
			    4: self.robot_provide_outcome,
				5: self.robot_move_back,
				6: self.user_move_back,
				7: self.robot_move_correct_token
		    }

		# get the function based on argument
		func = switcher.get(argument)

		# Execute the function
		return func()

def main():
	game = Game(task_length=5, n_max_attempt_per_token=4, timeout=15)
	user_id = raw_input("please, insert the id of the user:")
	path = os.path.abspath(__file__)
	dir_path = os.path.dirname(path)
	parent_dir_of_file = os.path.dirname(dir_path)
	path_name = parent_dir_of_file + "/log/" + user_id

	if not os.path.exists(path_name):
		os.makedirs(path_name)
	else:
		user_id = raw_input("The folder already exists, please remove it or create a new one:")
		path_name = os.getcwd() + "/log/" + user_id
		if not os.path.exists(path_name):
			os.makedirs(path_name)

	file_spec = path_name + "/log_spec.txt"
	file_gen = path_name + "/log_gen.txt"
	file_summary = path_name + "/log_summary.txt"

	log_spec = Log(file_spec)
	game.move_info_spec['token_id'] = "token_id"
	game.move_info_spec['from'] = "from"
	game.move_info_spec['to'] = "to"
	game.move_info_spec['robot_assistance'] = "robot_assistance"
	game.move_info_spec['react_time'] = "react_time"
	game.move_info_spec['elapsed_time'] = "elapsed_time"
	game.move_info_spec['attempt'] = "attempt"
	game.move_info_spec['sociable'] = "sociable"
	game.move_info_spec_vect.append(game.move_info_spec)
	log_spec.add_row_entry(game.move_info_spec)

	log_gen = Log(file_gen)
	game.move_info_gen['token_id'] = "token_id"
	game.move_info_gen['from'] = "from"
	game.move_info_gen['to'] = "to"
	game.move_info_gen['avg_robot_assistance_per_move'] = "avg_assistance_per_move"
	game.move_info_gen['cum_react_time'] = "cum_react_time"
	game.move_info_gen['cum_elapsed_time'] = "cum_elapsed_time"
	game.move_info_gen['attempt'] = "attempt"
	game.move_info_gen['sociable'] = "sociable"
	game.move_info_gen_vect.append(game.move_info_gen)
	log_gen.add_row_entry(game.move_info_gen)

	log_summary = Log(file_summary)
	game.move_info_summary["attempt"] = "n_attempt"
	game.move_info_summary["sociable"] = "n_sociable"
	game.move_info_summary["avg_lev_assistance"] = "lev_assistance"
	game.move_info_summary["react_time"] = "react_time"
	game.move_info_summary["elapsed_time"] = "elapsed_time"
	log_summary.add_row_entry(game.move_info_summary)
	sm = StateMachine(1)

	while game.get_n_correct_move()<game.task_length:

		if sm.CURRENT_STATE.value == sm.S_ROBOT_ASSIST.value:
			sm.robot_provide_assistance(game)
			game.avg_robot_assistance_per_move += game.robot_assistance

		elif sm.CURRENT_STATE.value == sm.S_USER_ACTION.value:
			print("Expected token ", game.solution[game.get_n_correct_move()])
			time_to_act = time.time()
			sm.user_action(game)
			game.total_elapsed_time += time.time() - time_to_act

		elif sm.CURRENT_STATE.value == sm.S_ROBOT_OUTCOME.value:
			#these are reported only because the variables are already reset when a correct move occurred
			sm.robot_provide_outcome(game)
			info = game.store_info_spec()
			log_spec.add_row_entry(info)
			#{'token_id':'', 'from':'', 'to':'', 'robot_assistance':'', 'react_time':'', 'elapsed_time':'', 'attempt':''}
			#if you have done the correct move or you reach the maximum number of attempts then store the data
			if (game.outcome == 1 or (game.outcome==0 and game.n_attempt_per_token==game.n_max_attempt_per_token)
					or (game.outcome==-1 and game.n_attempt_per_token==game.n_max_attempt_per_token)):
				log = game.store_info_gen()
				log_gen.add_row_entry(log)
				game.reset_counters()
			game.reset_detected_token()


	log = game.store_info_summary()
	log_summary.add_row_entry(log)

	for instance_spec in game.move_info_spec_vect:
		print(instance_spec)

	for instance_gen in game.move_info_gen_vect:
		print(instance_gen)

	print(game.total_elapsed_time)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass