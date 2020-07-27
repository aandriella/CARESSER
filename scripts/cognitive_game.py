import rospy
from board_state.msg import TokenMsg
from board_state.msg import BoardMsg


class Game(object):
	def __init__(self, task_length, n_max_attempt_per_token, timeout):
		rospy.init_node('big_hero', anonymous=True)
		# subscriber for getting info from the board
		rospy.Subscriber("/detected_move", TokenMsg, self.get_move_event_callback)
		rospy.Subscriber("/board_status", BoardMsg, self.get_board_event_callback)
		#get the objective of the exercise from the launch file
		self.objective = rospy.get_param("/objective")
		self.with_SOCIABLE = rospy.get_param("/SOCIABLE")
		#we need a sleep in order to give the system the time to get the board info
		self.current_board = []
		rospy.sleep(2)
		self.initial_board = self.get_board_event()
		self.task_length = task_length
		self.n_max_attempt_per_token = n_max_attempt_per_token
		self.solution = self.set_objective(self.task_length)
		self.timeout = timeout
		self.robot_assistance = 0
		self.avg_robot_assistance_per_move = 0
		self.outcome = 0
		#counters
		self.n_attempt_per_token = 1
		self.n_tot_sociable = 0
		self.n_sociable_per_token = 0
		self.n_mistakes = 0
		self.n_correct_move = 0
		#subscriber variables from detect_move
		self.detected_token = []
		self.picked = False
		self.placed = False
		self.moved_back = False
		#logger
		self.react_time_per_token_spec_t0 = 0.0
		self.react_time_per_token_gen_t0 = 0.0
		self.react_time_per_token_spec_t1 = 0.0
		self.react_time_per_token_gen_t1 = 0.0
		self.elapsed_time_per_token_gen_t0 = 0.0
		self.elapsed_time_per_token_spec_t0 = 0.0
		self.elapsed_time_per_token_gen_t1 = 0.0
		self.elapsed_time_per_token_spec_t1 = 0.0
		self.total_elapsed_time = 0.0
		self.move_info_gen = dict()
		self.move_info_spec = dict()
		self.move_info_summary = dict()
		self.move_info_gen_vect = list()
		self.move_info_spec_vect = list()
		self.move_info_summary_vect = list()

	def get_board_event_callback(self, msg):
		'''callback from the topic board_status to get the status of the current_board'''
		self.current_board = msg.data

	def get_board_event(self):
		'''This method returns what is listened by the subscriber'''
		return self.current_board

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
		board_ = self.current_board[:]
		#remove the empty cells from the current_board
		board_filtered = list(filter(lambda x: x!="0", board_))
		if self.objective == "ascending":
			self.solution = sorted(board_filtered)[:n_token]
		elif self.objective == "descending":
			self.solution = sorted(board_filtered, reverse=False)[:n_token]
		else:
			assert("Game is not defined contact the developer for integrating it")
		return self.solution

	#methods to access the variables outside the class
	def get_n_attempt_per_token(self):
		return self.n_attempt_per_token

	def get_n_max_attempt(self):
		return self.n_max_attempt

	def get_n_mistakes(self):
		return self.n_mistakes

	def get_n_solution(self):
		return self.n_solution

	def get_n_correct_move(self):
		return self.n_correct_move

	def reset_attempt_per_token(self):
		self.n_attempt_per_token = 0

	def set_n_attempt_per_token(self, value):
		self.n_attempt_per_token = value

	def set_n_mistakes(self, value):
		self.n_mistakes = value

	def set_n_correct_move(self, value):
		self.n_correct_move = value

	def set_n_solution(self, value):
		self.n_solution = value

	def set_n_max_attempt_per_token(self, value):
		self.n_max_attempt_per_token = value

	def add_info_gen_vect(self, dict):
		self.move_info_gen_vect.append(dict.copy())

	def add_info_spec_vect(self, dict):
		self.move_info_spec_vect.append(dict.copy())

	def store_info_spec(self):
		self.move_info_spec['token_id'] = self.detected_token[0]
		self.move_info_spec['from'] = self.detected_token[1]
		self.move_info_spec['to'] = self.detected_token[2]
		self.move_info_spec['robot_assistance'] = self.robot_assistance
		self.move_info_spec['react_time'] = round(self.react_time_per_token_spec_t1, 3)
		self.move_info_spec['elapsed_time'] = round(self.elapsed_time_per_token_spec_t1, 3)
		self.move_info_spec['attempt'] = self.n_attempt_per_token
		self.move_info_spec['sociable'] = self.n_sociable_per_token
		self.add_info_spec_vect(self.move_info_spec)
		return self.move_info_spec

	def store_info_gen(self):
		self.move_info_gen['token_id'] = self.detected_token[0]
		self.move_info_gen['from'] = self.detected_token[1]
		self.move_info_gen['to'] = self.detected_token[2]
		self.move_info_gen['avg_robot_assistance_per_move'] = round(
					self.avg_robot_assistance_per_move / self.n_attempt_per_token, 3)
		self.move_info_gen['cum_react_time'] = round(self.react_time_per_token_gen_t1, 3)
		self.move_info_gen['cum_elapsed_time'] = round(self.elapsed_time_per_token_gen_t1, 3)
		self.move_info_gen['attempt'] = self.n_attempt_per_token
		self.move_info_gen['sociable'] = self.n_sociable_per_token
		self.add_info_gen_vect(self.move_info_gen)
		return self.move_info_gen

	def store_info_summary(self):
		self.move_info_summary["attempt"] = "n_attempt"
		self.move_info_summary["avg_lev_assistance"] = "lev_assistance"
		self.move_info_summary["react_time"] = "react_time"
		self.move_info_summary["elapsed_time"] = "elapsed_time"
		self.move_info_summary["attempt"] = sum([elem['attempt'] for elem in self.move_info_gen_vect])
		self.move_info_summary["sociable"] = sum([elem['sociable'] for elem in self.move_info_gen_vect])
		self.move_info_summary["avg_lev_assistance"] = sum(
			[elem['avg_robot_assistance_per_move'] for elem in self.move_info_gen_vect]) / self.task_length
		self.move_info_summary["react_time"] = sum([elem['cum_react_time'] for elem in self.move_info_gen_vect])
		self.move_info_summary["elapsed_time"] = sum([elem['cum_elapsed_time'] for elem in self.move_info_gen_vect])
		return self.move_info_summary

	def reset_counters(self):
		self.react_time_per_token_spec_t1 = 0
		self.react_time_per_token_gen_t1 = 0
		self.react_time_per_token_spec_t0 = 0
		self.react_time_per_token_gen_t0 = 0
		self.elapsed_time_per_token_spec_t1 = 0
		self.elapsed_time_per_token_gen_t1 = 0
		self.elapsed_time_per_token_spec_t0 = 0
		self.elapsed_time_per_token_gen_t0 = 0
		self.avg_robot_assistance_per_move = 0
		self.n_sociable_per_token = 0
		self.n_correct_move += 1
		self.n_attempt_per_token = 1
		self.set_n_correct_move(self.n_correct_move)
		self.set_n_attempt_per_token(self.n_attempt_per_token)

	def reset_detected_token(self):
		self.detected_token = []
		self.picked = False
		self.placed = False
