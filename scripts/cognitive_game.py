import rospy
from board_state.msg import TokenMsg
from board_state.msg import BoardMsg


class Game(object):
  def __init__(self, board_size, task_length, n_max_attempt_per_token, timeout, objective,
               bn_game_state, bn_attempt, bn_caregiver_feedback, bn_caregiver_assistance,
               bn_user_react_time, bn_user_action
               ):

    rospy.init_node('big_hero', anonymous=True)
    # subscriber for getting info from the board
    rospy.Subscriber("/detected_move", TokenMsg, self.get_move_event_callback)
    rospy.Subscriber("/board_status", BoardMsg, self.get_board_event_callback)
    #get the objective of the exercise from the launch file
    self.objective = objective
    #we need a sleep in order to give the system the time to get the board info
    self.current_board = []
    rospy.sleep(2)
    self.initial_board = self.get_board_event()
    self.task_length = task_length
    self.n_max_attempt_per_token = n_max_attempt_per_token
    self.solution = self.set_objective(self.task_length)
    self.timeout = timeout
    self.caregiver_assistance = 0
    self.robot_feedback = 0
    self.avg_caregiver_assistance_per_move = 0
    self.outcome = 0
    self.width = board_size[0]
    self.height = board_size[1]
    #counters
    self.n_attempt_per_token = 1
    self.n_timeout_per_token = 0
    self.n_tot_sociable = 0
    self.n_sociable_per_token = 0
    self.n_mistakes = 0
    self.n_correct_move = 0
    #subscriber variables from detect_move
    self.detected_token = ()
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
    self.bn_varibles = dict()

    self.move_info_gen_vect = list()
    self.move_info_spec_vect = list()
    self.move_info_summary_vect = list()
    self.bn_varibles_vect = list()

    self.bn_user_action = bn_user_action
    self.bn_user_react_time = bn_user_react_time
    self.bn_game_state = bn_game_state
    self.bn_attempt = bn_attempt
    self.bn_caregiver_feedback = bn_caregiver_feedback
    self.bn_caregiver_assistance = bn_caregiver_assistance

    #This is for user action

    self.attempt_counter_per_action = [[0 for i in range(len(self.bn_attempt.values()))] for j in range(len(self.bn_user_action.values()))]
    self.game_state_counter_per_action = [[0 for i in range(len(self.bn_game_state.values()))] for j in
                                     range(len(self.bn_user_action.values()))]

    self.caregiver_feedback_per_action = [[0 for i in range(len(self.bn_caregiver_feedback.values()))] for j in
                                 range(len(self.bn_user_action.values()))]
    self.caregiver_assistance_per_action = [[0 for i in range(len(self.bn_caregiver_assistance.values()))] for j in
                                   range(len(self.bn_user_action.values()))]

    #This is for user react time
    self.attempt_counter_per_react_time = [[0 for i in range(len(self.bn_attempt.values()))] for j in
                                      range(len(self.bn_user_react_time.values()))]
    self.game_state_counter_per_react_time = [[0 for i in range(len(self.bn_game_state.values()))] for j in
                                         range(len(self.bn_user_react_time.values()))]
    self.caregiver_feedback_per_react_time = [[0 for i in range(len(self.bn_caregiver_feedback.values()))] for j in
                                     range(len(self.bn_user_react_time.values()))]
    self.caregiver_assistance_per_react_time = [[0 for i in range(len(self.bn_caregiver_assistance.values()))] for j in
                                       range(len(self.bn_user_react_time.values()))]

    #This is for caregiver assistance
    self.game_state_counter_per_caregiver_assistance = [[0 for i in range(len(self.bn_game_state.values()))] for j in
                                               range(len(self.bn_caregiver_assistance.values()))]
    self.attempt_counter_per_caregiver_assistance = [[0 for i in range(len(self.bn_attempt.values()))] for j in
                                            range(len(self.bn_caregiver_assistance.values()))]

    #This is for caregiver feedback
    self.game_state_counter_per_caregiver_feedback = [[0 for i in range(len(self.bn_game_state.values()))] for j in
                                             range(len(self.bn_caregiver_feedback.values()))]
    self.attempt_counter_per_caregiver_feedback = [[0 for i in range(len(self.bn_attempt.values()))] for j in
                                          range(len(self.bn_caregiver_feedback.values()))]

    self.bn_dict_vars_user_action = {'attempt':self.attempt_counter_per_action,
						 'game_state':self.game_state_counter_per_action,
						 'agent_feedback':self.caregiver_feedback_per_action,
						 'agent_assistance':self.caregiver_assistance_per_action}

    self.bn_dict_vars_user_react_time = {'attempt':self.attempt_counter_per_react_time,
						 'game_state':self.game_state_counter_per_react_time,
						 'agent_feedback':self.caregiver_feedback_per_react_time,
						 'agent_assistance':self.caregiver_assistance_per_react_time}
    self.bn_dict_vars_caregiver_assistance = {'game_state':self.game_state_counter_per_caregiver_assistance,
						 'attempt':self.attempt_counter_per_caregiver_assistance}
    self.bn_dict_vars_caregiver_feedback = {'game_state':self.game_state_counter_per_caregiver_feedback,
						'attempt':self.attempt_counter_per_caregiver_feedback}


  def check_board(self):
    '''this method checks if all the tokens are on the board if notask the user to place it correctly'''
    n_tokens_on_board = self.current_board.count('0')
    if n_tokens_on_board!= self.task_length:
      print("ask the caregiver to do something")
      return False
    else:
      return True


  def map_user_action(self, outcome):
    if outcome == -1:
      return self.bn_user_action['wrong']
    elif outcome == 0:
      return self.bn_user_action['timeout']
    elif outcome == 1:
      return self.bn_user_action['correct']

  def map_react_time(self):
    '''
    discretise the react time in 3 bins [slow, normal, fast] 0, 1a, 2
    Args:

    Return:
      0 if slow 1a if normal 2 if fast
    '''
    if self.react_time_per_token_spec_t1>0 and self.react_time_per_token_spec_t1<self.bn_user_react_time['fast']:
      return 2
    elif self.react_time_per_token_spec_t1>=self.bn_user_react_time['fast'] and self.react_time_per_token_spec_t1<self.bn_user_react_time['normal']:
      return 1
    elif self.react_time_per_token_spec_t1 >= self.bn_user_react_time['normal']:
      return 0

  def map_game_state(self):
    '''
    get the game state : BEG, MIDDLE and END
    Return:
    the id of the game state
    '''
    if self.n_correct_move<self.bn_game_state['beg']:
      return 0
    elif self.n_correct_move>=self.bn_game_state['beg'] and self.n_correct_move<self.bn_game_state['mid']:
      return 1
    else:
      return 2

  def get_token_sol(self):
    '''This method returns the correct token to move its initial location its final location'''
    token_id = self.solution[self.n_correct_move]
    token_from = [index for index in range(len(self.initial_board)) if
                     self.initial_board[index] == token_id].pop() + 1
    token_to = self.n_correct_move + 1
    return (token_id, token_from, token_to)

  def get_token_init_loc(self, token_id):
    '''This method returns the  token   initial location on the board'''
    token_from = [index for index in range(len(self.initial_board)) if
                     self.initial_board[index] == token_id].pop() + 1
    return (token_id, token_from)

  def get_board_event_callback(self, msg):
    '''callback from the topic board_status to get the status of the current_board'''
    self.current_board = msg.data

  def get_board_event(self):
    '''This method returns what is listened by the subscriber'''
    return self.current_board

  def get_move_event_callback(self, msg):
    '''callback from the topic detected_move to get the detected move if so'''
    if msg.detected_token == []:
      self.detected_token = ("", 0, 0)
    else:
      self.detected_token = (msg.detected_token[0], int(msg.detected_token[1]), int(msg.detected_token[2]))
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
    board_filtered = [int(i) for i in list(filter(lambda x: x!="0", board_))]
    even_number_list = [num for num in board_filtered if int(num) % 2 == 0]
    odd_number_list = [num for num in board_filtered if int(num) % 2 != 0]
    if self.objective == "ascending":
      self.solution = [str(num) for num in sorted(board_filtered)[:n_token]]
    elif self.objective == "ascending_even":
      self.solution = [str(num) for num in sorted(even_number_list)[:n_token]]
    elif self.objective == "ascending_odd":
      self.solution = [str(num) for num in sorted(odd_number_list)[:n_token]]
    elif self.objective == "descending":
      self.solution = [str(num) for num in sorted(board_filtered, reverse=True)[:n_token]]
    elif self.objective == "descending_even":
      self.solution = [str(num) for num in sorted(even_number_list, reverse=True)[:n_token]]
    elif self.objective == "descending_odd":
      self.solution = [str(num) for num in sorted(odd_number_list, reverse=True)[:n_token]]
    else:
      assert("Game is not defined contact the developer for integrating it")
      exit(-1)
    return self.solution

  #methods to access the variables outside the class
  def get_n_attempt_per_token(self):
    return self.n_attempt_per_token

  def get_n_max_attempt(self):
    return self.n_max_attempt

  def get_n_mistakes(self):
    return self.n_mistakes

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

  def set_n_max_attempt_per_token(self, value):
    self.n_max_attempt_per_token = value

  def add_info_gen_vect(self, dict):
    self.move_info_gen_vect.append(dict.copy())

  def add_info_spec_vect(self, dict):
    self.move_info_spec_vect.append(dict.copy())

  def add_info_bn_variables(self, dict):
    self.bn_varibles_vect.append(dict.copy())


  def store_bn_variables(self, outcome):
    self.bn_varibles['game_state'] = self.map_game_state()
    self.bn_varibles['attempt'] = self.n_attempt_per_token
    self.bn_varibles['user_react_time'] = self.map_react_time()
    self.bn_varibles['agent_assistance'] = 0
    self.bn_varibles['agent_feedback'] = 0
    self.bn_varibles['user_action'] = self.map_user_action(outcome)
    self.bn_varibles['user_reactivity'] = 0
    self.bn_varibles['user_memory'] = 0
    self.add_info_bn_variables(self.bn_varibles)
    return self.bn_varibles

  def store_info_spec(self, outcome):
    #timeout
    if outcome==0:
      self.move_info_spec['game_state'] = self.map_game_state()
      self.move_info_spec['user_action'] = self.map_user_action(outcome)
      self.move_info_spec['user_react_time'] = self.map_react_time()
      self.move_info_spec['token_id'] = ""
      self.move_info_spec['from'] = ""
      self.move_info_spec['to'] = ""
      self.move_info_spec['caregiver_assistance'] = self.caregiver_assistance
      self.move_info_spec['react_time'] = self.timeout
      self.move_info_spec['elapsed_time'] = 0
      self.move_info_spec['attempt'] = self.n_attempt_per_token
      self.move_info_spec['caregiver_feedback'] = self.n_sociable_per_token
      self.move_info_spec['timeout'] = self.n_timeout_per_token
      self.add_info_spec_vect(self.move_info_spec)
    else:
      self.move_info_spec['game_state'] = self.map_game_state()
      self.move_info_spec['user_action'] = self.map_user_action(outcome)
      self.move_info_spec['user_react_time'] = self.map_react_time()
      self.move_info_spec['token_id'] = self.detected_token[0]
      self.move_info_spec['from'] = self.detected_token[1]
      self.move_info_spec['to'] = self.detected_token[2]
      self.move_info_spec['caregiver_assistance'] = self.caregiver_assistance
      self.move_info_spec['react_time'] = round(self.react_time_per_token_spec_t1, 3)
      self.move_info_spec['elapsed_time'] = round(self.elapsed_time_per_token_spec_t1, 3)
      self.move_info_spec['attempt'] = self.n_attempt_per_token
      self.move_info_spec['caregiver_feedback'] = self.n_sociable_per_token
      self.move_info_spec['timeout'] = self.n_timeout_per_token
      self.add_info_spec_vect(self.move_info_spec)

    self.attempt_counter_per_action[self.map_user_action(outcome)][self.n_attempt_per_token-1] += 1
    self.game_state_counter_per_action[self.map_user_action(outcome)][self.map_game_state()] += 1
    self.caregiver_feedback_per_action[self.map_user_action(outcome)][self.robot_feedback] += 1
    self.caregiver_assistance_per_action[self.map_user_action(outcome)][self.caregiver_assistance] += 1

    self.attempt_counter_per_react_time[self.map_react_time()][self.n_attempt_per_token-1] += 1
    self.game_state_counter_per_react_time[self.map_react_time()][self.map_game_state()] += 1
    self.caregiver_feedback_per_react_time[self.map_react_time()][self.robot_feedback] += 1
    self.caregiver_assistance_per_react_time[self.map_react_time()][self.caregiver_assistance] += 1

    self.game_state_counter_per_caregiver_assistance[self.caregiver_assistance][self.map_game_state()] += 1
    self.attempt_counter_per_caregiver_assistance[self.caregiver_assistance][self.n_attempt_per_token-1] += 1
    self.game_state_counter_per_caregiver_feedback[self.robot_feedback][self.map_game_state()] += 1
    self.attempt_counter_per_caregiver_feedback[self.robot_feedback][self.n_attempt_per_token-1] += 1


    return self.move_info_spec

  def store_info_gen(self):
    self.move_info_gen['token_id'] = self.detected_token[0]
    self.move_info_gen['from'] = self.detected_token[1]
    self.move_info_gen['to'] = self.detected_token[2]
    self.move_info_gen['avg_caregiver_assistance_per_move'] = round(
      self.avg_caregiver_assistance_per_move / self.n_attempt_per_token, 3)
    self.move_info_gen['cum_react_time'] = round(self.react_time_per_token_gen_t1, 3)
    self.move_info_gen['cum_elapsed_time'] = round(self.elapsed_time_per_token_gen_t1, 3)
    self.move_info_gen['attempt'] = self.n_attempt_per_token
    self.move_info_gen['timeout'] = self.n_timeout_per_token
    self.move_info_gen['caregiver_feedback'] = self.n_sociable_per_token
    self.add_info_gen_vect(self.move_info_gen)
    return self.move_info_gen

  def store_info_summary(self):
    entry_log_summary = ["n_attempt", "n_sociable", "avg_lev_assistance", "tot_react_time", "tot_elapsed_time"]

    self.move_info_summary["n_attempt"] = sum([elem['attempt'] for elem in self.move_info_gen_vect])
    self.move_info_summary["n_timeout"] = sum([elem['timeout'] for elem in self.move_info_gen_vect])
    self.move_info_summary["n_sociable"] = sum([elem['caregiver_feedback'] for elem in self.move_info_gen_vect])
    self.move_info_summary["avg_lev_assistance"] = sum(
      [elem['avg_caregiver_assistance_per_move'] for elem in self.move_info_gen_vect]) / self.task_length
    self.move_info_summary["tot_react_time"] = sum([elem['cum_react_time'] for elem in self.move_info_gen_vect])
    self.move_info_summary["tot_elapsed_time"] = sum([elem['cum_elapsed_time'] for elem in self.move_info_gen_vect])
    return self.move_info_summary

  def reset_counters_spec(self):
    self.react_time_per_token_spec_t1 = 0
    self.react_time_per_token_spec_t0 = 0
    self.elapsed_time_per_token_spec_t1 = 0
    self.elapsed_time_per_token_spec_t0 = 0
    self.avg_caregiver_assistance_per_move = 0

  def reset_counters_gen(self):
    self.react_time_per_token_gen_t1 = 0
    self.react_time_per_token_gen_t0 = 0
    self.elapsed_time_per_token_gen_t1 = 0
    self.elapsed_time_per_token_gen_t0 = 0


  def reset_detected_token(self):
    self.detected_token = []
    self.picked = False
    self.placed = False
