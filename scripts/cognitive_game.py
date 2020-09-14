import rospy
from board_state.msg import TokenMsg
from board_state.msg import BoardMsg


class Game(object):
  def __init__(self, board_size, task_length, n_max_attempt_per_token, timeout, objective,
               game_state):
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
    self.robot_assistance = 0
    self.avg_robot_assistance_per_move = 0
    self.outcome = 0
    self.width = board_size[0]
    self.height = board_size[1]
    self.game_state = game_state
    #counters
    self.n_attempt_per_token = 1
    self.n_timeout_per_token = 0
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

    #TODO For the BN Simulator, initialise this variable according to the simulator
    # self.caregiver_assistance_per_action = [][]
    # self.caregiver_feedback_per_action = [][]
    # self.game_state_per_action = [][]
    # self.attempt_per_action = [][]
    # self.caregiver_assistance_per_action = [][]
    # self.caregiver_feedback_per_action = [][]
    # self.game_state_per_action = [][]
    # self.attempt_per_action = [][]

  def get_game_state(self):
    '''
    get the game state : BEG, MIDDLE and END
    Return:
    the id of the game state
    '''
    if self.n_correct_move<self.game_state['beg']:
      return 0
    elif self.n_correct_move>=self.game_state['beg'] and self.n_correct_move<self.game_state['mid']:
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

  def store_info_spec(self, outcome):
    #timeout
    if outcome==0:
      self.move_info_spec['game_state'] = self.get_game_state()
      self.move_info_spec['token_id'] = ""
      self.move_info_spec['from'] = ""
      self.move_info_spec['to'] = ""
      self.move_info_spec['robot_assistance'] = self.robot_assistance
      self.move_info_spec['react_time'] = self.timeout
      self.move_info_spec['elapsed_time'] = 0
      self.move_info_spec['attempt'] = self.n_attempt_per_token
      self.move_info_spec['sociable'] = self.n_sociable_per_token
      self.move_info_spec['timeout'] = self.n_timeout_per_token
      self.add_info_spec_vect(self.move_info_spec)
    else:
      self.move_info_spec['game_state'] = self.get_game_state()
      self.move_info_spec['token_id'] = self.detected_token[0]
      self.move_info_spec['from'] = self.detected_token[1]
      self.move_info_spec['to'] = self.detected_token[2]
      self.move_info_spec['robot_assistance'] = self.robot_assistance
      self.move_info_spec['react_time'] = round(self.react_time_per_token_spec_t1, 3)
      self.move_info_spec['elapsed_time'] = round(self.elapsed_time_per_token_spec_t1, 3)
      self.move_info_spec['attempt'] = self.n_attempt_per_token
      self.move_info_spec['sociable'] = self.n_sociable_per_token
      self.move_info_spec['timeout'] = self.n_timeout_per_token
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
    self.move_info_gen['timeout'] = self.n_timeout_per_token
    self.move_info_gen['sociable'] = self.n_sociable_per_token
    self.add_info_gen_vect(self.move_info_gen)
    return self.move_info_gen

  def store_info_summary(self):
    entry_log_summary = ["n_attempt", "n_sociable", "avg_lev_assistance", "tot_react_time", "tot_elapsed_time"]

    self.move_info_summary["n_attempt"] = sum([elem['attempt'] for elem in self.move_info_gen_vect])
    self.move_info_summary["n_timeout"] = sum([elem['timeout'] for elem in self.move_info_gen_vect])
    self.move_info_summary["n_sociable"] = sum([elem['sociable'] for elem in self.move_info_gen_vect])
    self.move_info_summary["avg_lev_assistance"] = sum(
      [elem['avg_robot_assistance_per_move'] for elem in self.move_info_gen_vect]) / self.task_length
    self.move_info_summary["tot_react_time"] = sum([elem['cum_react_time'] for elem in self.move_info_gen_vect])
    self.move_info_summary["tot_elapsed_time"] = sum([elem['cum_elapsed_time'] for elem in self.move_info_gen_vect])
    return self.move_info_summary

  def reset_counters_spec(self):
    self.react_time_per_token_spec_t1 = 0
    self.react_time_per_token_spec_t0 = 0
    self.elapsed_time_per_token_spec_t1 = 0
    self.elapsed_time_per_token_spec_t0 = 0
    self.avg_robot_assistance_per_move = 0

  def reset_counters_gen(self):
    self.react_time_per_token_gen_t1 = 0
    self.react_time_per_token_gen_t0 = 0
    self.elapsed_time_per_token_gen_t1 = 0
    self.elapsed_time_per_token_gen_t0 = 0


  def reset_detected_token(self):
    self.detected_token = []
    self.picked = False
    self.placed = False
