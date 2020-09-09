#!/usr/bin/python
'''
This is the main class for running the entire game framework
'''
# import modules and classes
from log import Log
from cognitive_game import Game
from robot_behaviour.robot_reproducer import Robot
from robot_behaviour.speech_reproducer import Speech
from robot_behaviour.face_reproducer import Face
from robot_behaviour.gesture_reproducer import Gesture
# import from libraries
import enum
import random
import time
import os

# import from ros
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

  def robot_provide_assistance(self, game, robot):
    '''
    Robot action of assistance combining speech and gesture
    Args:
      game: the game instance
      robot: robot instance to perform an action
    Return:
       True when the action has been completed
    '''

    print("R_ASSISTANCE")
    #recall all the information you may need for providing assistance
    token_sol = game.get_token_sol()
    tokens_subset = game.get_subset(3)
    token_row = game.get_token_row()
    game.robot_assistance = 2#random.randint(0, 5)
    success = robot.action["assistance"].__call__(lev_id=game.robot_assistance, row=token_row, counter=game.n_attempt_per_token-1, token=token_sol, facial_expression="neutral", tokens=tokens_subset)

    self.b_robot_assist_finished = True
    self.b_robot_reengaged_user == False
    self.CURRENT_STATE = self.S_USER_ACTION
    return self.b_robot_assist_finished

  # def robot_provide_feedback(self):
  #   '''
  #   The robot provides a feddback on the grasped token
  #   :return:
  #   '''
  #   print("R_FEEDBACK")
  #   self.b_robot_feedback_finished = True
  #   self.CURRENT_STATE = self.S_USER_PLACE
  #   return self.b_robot_feedback_finished

  # def robot_reengage_user(self):
  #   '''As the timeout occurred the robot reengages the user'''
  #   print("RE_ENGAGE USER")
  #   self.CURRENT_STATE = self.S_USER_ACTION
  #   self.b_robot_reengaged_user = True

  def update_counters(self, game):
    '''
        Auxiliary function to update the counters based on the outcome
        :param self:
        :return:
        '''
    print("Update_counters")

    if game.moved_back:
      print("token has been moved back, DO nothing")
      game.n_mistakes += 1
      game.n_attempt_per_token += 1
      game.set_n_attempt_per_token(game.n_attempt_per_token)
      game.set_n_mistakes(game.n_mistakes)

      if game.n_attempt_per_token > game.n_max_attempt_per_token:
        print("Max attempt reached")
        game.set_n_correct_move(game.get_n_correct_move() + 1)
        game.set_n_attempt_per_token(1)
        game.n_sociable_per_token = 0
        game.n_timeout_per_token = 0

    elif game.outcome == 0:
      game.n_timeout_per_token += 1
      game.n_mistakes += 1
      game.n_attempt_per_token += 1
      game.set_n_attempt_per_token(game.n_attempt_per_token)
      game.set_n_mistakes(game.n_mistakes)

      # check if the user reached his max number of attempts
      if game.n_attempt_per_token > game.n_max_attempt_per_token:
        print("Max attempt reached")
        game.set_n_correct_move(game.get_n_correct_move() + 1)
        game.set_n_attempt_per_token(1)
        game.n_sociable_per_token = 0
        game.n_timeout_per_token = 0

    # get current move and check if it is the one expeceted in the solution list
    elif game.outcome == 1:
      game.set_n_correct_move(game.get_n_correct_move() + 1)
      game.set_n_attempt_per_token(1)
      game.n_sociable_per_token = 0
      game.n_timeout_per_token = 0

    elif game.outcome == -1:
      print("wrong_solution")
      game.n_mistakes += 1
      game.n_attempt_per_token += 1
      game.set_n_attempt_per_token(game.n_attempt_per_token)
      game.set_n_mistakes(game.n_mistakes)

      # check if the user reached his max number of attempts
      if game.n_attempt_per_token > game.n_max_attempt_per_token:
        print("Max attempt reached")
        game.set_n_correct_move(game.get_n_correct_move() + 1)
        game.set_n_attempt_per_token(1)
        game.n_sociable_per_token = 0
        game.n_timeout_per_token = 0


  def robot_provide_outcome(self, game, robot):
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
        self.robot_move_correct_token(game, robot)
        game.outcome = 1

    elif game.detected_token == ("",0,0):
      # check if the user reached his max number of attempts
      if game.n_attempt_per_token >= game.n_max_attempt_per_token:
        print("Max attempt reached")
        self.S_ROBOT_MOVE_CORRECT_TOKEN = True
        self.b_user_reached_max_attempt = True
        self.robot_move_correct_token(game, robot)
        game.outcome = 1
      else:
        print("timeout")
        robot.action["timeout"].__call__(counter=game.n_attempt_per_token - 1, facial_expression="sad")
        game.outcome = 0
        self.CURRENT_STATE = self.S_ROBOT_ASSIST

    # get current move and check if it is the one expeceted in the solution list
    elif game.detected_token[0] == game.solution[game.n_correct_move] \
        and game.detected_token[2] == (game.solution.index(game.detected_token[0]) + 1):
      robot.action["congrats"].__call__(counter=game.n_attempt_per_token-1, facial_expression="happy")
      game.outcome = 1
      print("correct_solution ", game.get_n_correct_move())
      self.CURRENT_STATE = self.S_ROBOT_ASSIST

    elif game.detected_token[0] != game.solution[game.n_correct_move] \
        or game.detected_token[2] != game.solution.index(game.detected_token[0]) + 1:
      game.outcome = -1
      print("wrong_solution")
      robot.action["compassion"].__call__(counter=game.n_attempt_per_token-1, facial_expression="sad")
      self.CURRENT_STATE = self.S_ROBOT_MOVE_TOKEN_BACK
      #if robot replace with
      self.robot_move_back(game, robot)
      #self.user_move_back(game, robot)

      # check if the user reached his max number of attempts
      if game.n_attempt_per_token >= game.n_max_attempt_per_token:
        print("Max attempt reached")
        self.S_ROBOT_MOVE_CORRECT_TOKEN = True
        self.b_user_reached_max_attempt = True
        self.robot_move_correct_token(game, robot)
        game.outcome = 1

    self.b_robot_outcome_finished = True
    return self.b_robot_outcome_finished

  def robot_move_correct_token(self, game, robot):
    print("Robot moves the correct token as the user reached the max number of attempts")
    # get the current solution
    token = game.get_token_sol()
    success = robot.action["max_attempt"].__call__(token=token, counter=game.n_attempt_per_token-1, facial_expression="sad")
    #while(game.detected_token != (token_id, token_from, token_to)):
    #  pass
    #print("Robot moved the token in the correct location")
    
    input = raw_input("move the token in the correct position and press a button")
    self.CURRENT_STATE = self.S_ROBOT_OUTCOME
    self.b_robot_moved_correct_token = True
    return self.b_robot_moved_correct_token

  def robot_move_back(self, game, robot):
    # user moved the token in an incorrect location
    # robot moved it back
    robot.cancel_action()
    token = game.detected_token
    success = robot.action["move_back"].__call__(who="robot", token=token, counter=game.n_attempt_per_token-1, facial_expression="neutral")
    while(game.detected_token != token):
      pass
    print("Robot moved back the token in its initial location")
    self.CURRENT_STATE = self.S_ROBOT_ASSIST
    self.b_robot_moved_token_back = True
    return self.b_robot_moved_token_back

  def user_move_back(self, game, robot):
    # user moved the token in an incorrect location
    # robot moved it back
    # get the initial location of the placed token and move back there
    token_id, token_from, token_to = game.detected_token
    success = robot.action["move_back"].__call__(who="user", token=game.get_token_sol(), counter=game.n_attempt_per_token-1, facial_expression="neutral")
    while (game.detected_token != (token_id, token_to, token_from)):
      pass
    print("User moved back the token in its initial location")
    self.CURRENT_STATE = self.S_ROBOT_ASSIST
    self.b_user_moved_token_back = True
    return self.b_user_moved_token_back

  def user_action(self, game, robot):
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
      elapsed_time = current_time - game.react_time_per_token_spec_t0
      if elapsed_time < game.timeout:
        return True
      else:
        return False

    def user_pick_token(sm, game, robot):
      game.react_time_per_token_spec_t0 = time.time()
      print("U_PICK")
      sm.CURRENT_STATE = sm.S_ROBOT_FEEDBACK
      detected_token, picked, _, _ = game.get_move_event()
      while (not picked and (detected_token == ("",0,0))):
        if check_move_timeout(game):
          detected_token, picked, _, _ = game.get_move_event()
        else:
          sm.b_user_reached_timeout = True
          game.react_time_per_token_spec_t0 = time.time()
          return False
      if robot.get_action_state() == 0:
        game.react_time_per_token_spec_t0 = time.time()
      game.elapsed_time_per_token_spec_t0 = time.time()
      game.react_time_per_token_spec_t1 = time.time() - game.react_time_per_token_spec_t0
      game.react_time_per_token_gen_t1 += game.react_time_per_token_spec_t1
      sm.b_user_picked_token = True
      return sm.b_user_picked_token

    def robot_provide_feedback(sm, game, robot):
      robot.cancel_action()
      print("R_FEEDBACK")
      game.n_sociable_per_token += 1
      game.n_tot_sociable += 1
      if game.detected_token[0] == game.solution[game.n_correct_move]:
        robot.action["pick"].__call__(positive=True, counter=game.n_attempt_per_token-1, facial_expression="happy")
      else:
        robot.action["pick"].__call__(positive=False, counter=game.n_attempt_per_token-1, facial_expression="confused")
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
      # check where the user will place the token
      def user_place_token_back(sm):
        print("U_PLACE_BACK")
        sm.CURRENT_STATE = sm.S_ROBOT_OUTCOME
        sm.b_user_placed_token_back = True
        game.elapsed_time_per_token_spec_t1 = time.time() - game.elapsed_time_per_token_spec_t0
        game.elapsed_time_per_token_gen_t1 += game.elapsed_time_per_token_spec_t1
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

      # here we check whether a token has been picked, and where it has been placed
      detected_token, picked, placed, moved_back = game.get_move_event()
      while (not placed):
        detected_token, _, placed, moved_back = game.get_move_event()
      if (placed and moved_back):
        sm.CURRENT_STATE = sm.S_ROBOT_OUTCOME
        return user_place_token_back(sm)
      elif (placed and not moved_back):
        sm.CURRENT_STATE = sm.S_ROBOT_OUTCOME
        return user_place_token_sol(sm)
      else:
        assert "Unexpected state"

    self.CURRENT_STATE = self.S_USER_ACTION
    # if the user picks a token and SOCIABLE is active
    if user_pick_token(self, game, robot):
      if game.with_SOCIABLE:
        if robot_provide_feedback(self, game, robot):
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
      print("user_pick_token==False-expect_timeout")
      self.CURRENT_STATE = self.S_ROBOT_OUTCOME

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
  language = rospy.get_param("/language")
  config_path = rospy.get_param("/config_path")
  sentences_file = ""
  if language == "en_GB":
    sentences_file = config_path+"/sentences_"+language
  elif language == "es":
    sentences_file =  config_path+"/sentences_"+language
  elif language == "cat":
    sentences_file =  config_path+"/sentences_"+language
  user_id = rospy.get_param("/user_id")
  with_SOCIABLE = rospy.get_param("/sociable")
  objective = rospy.get_param("/objective")

  # we create the game instance
  game = Game(board_size=(5, 4), task_length=5, n_max_attempt_per_token=4, timeout=20, objective=objective, with_SOCIABLE=with_SOCIABLE)
  # we create the robot instance
  speech = Speech(language)
  face = Face()
  gesture = Gesture()
  tiago_robot = Robot(speech, sentences_file, face, gesture)

  #user_id = raw_input("please, insert the id of the user:")
  path = os.path.abspath(__file__)
  dir_path = os.path.dirname(path)
  parent_dir_of_file = os.path.dirname(dir_path)
  path_name = parent_dir_of_file + "/robot_in_the_loop/log/" + str(user_id)

  if not os.path.exists(path_name):
    os.makedirs(path_name)
  else:
    user_id = raw_input("The folder already exists, please remove it or create a new one:")
    path_name = parent_dir_of_file + "/robot_in_the_loop/log/" + user_id
    if not os.path.exists(path_name):
      os.makedirs(path_name)


  file_spec = path_name + "/log_spec.txt"
  file_gen = path_name + "/log_gen.txt"
  file_summary = path_name + "/log_summary.txt"

  log_spec = Log(file_spec)
  entry_log_spec = ['token_id', 'from', 'to',
                    'robot_assistance', "react_time",
                    'elapsed_time', "attempt", "timeout", "sociable"]
  game.move_info_spec = {e: e for e in entry_log_spec}
  game.move_info_spec_vect.append(game.move_info_spec)
  log_spec.add_row_entry(game.move_info_spec)

  log_gen = Log(file_gen)
  entry_log_gen = ["token_id", "from", "to", "avg_robot_assistance_per_move", "cum_react_time",
                   "cum_elapsed_time", "attempt", "timeout", "sociable"]
  game.move_info_gen = {e: e for e in entry_log_gen}
  game.move_info_gen_vect.append(game.move_info_gen)
  log_gen.add_row_entry(game.move_info_gen)

  log_summary = Log(file_summary)
  entry_log_summary = ["n_attempt", "n_timeout",  "n_sociable", "avg_lev_assistance", "tot_react_time", "tot_elapsed_time"]
  game.move_info_summary = {e: e for e in entry_log_summary}
  log_summary.add_row_entry(game.move_info_summary)
  sm = StateMachine(1)

  #tiago_robot.instruction()

  while game.get_n_correct_move() < game.task_length:

    if sm.CURRENT_STATE.value == sm.S_ROBOT_ASSIST.value:
      sm.robot_provide_assistance(game, tiago_robot)
      game.avg_robot_assistance_per_move += game.robot_assistance

    elif sm.CURRENT_STATE.value == sm.S_USER_ACTION.value:
      print("Expected token ", game.solution[game.get_n_correct_move()])
      time_to_act = time.time()
      #game.with_SOCIABLE = random.randint(0,1)
      sm.user_action(game, tiago_robot)
      game.total_elapsed_time += time.time() - time_to_act

    elif sm.CURRENT_STATE.value == sm.S_ROBOT_OUTCOME.value:
      # these are reported only because the variables are already reset when a correct move occurred
      sm.robot_provide_outcome(game, tiago_robot)

      if (game.outcome == 1 or (game.outcome == 0 and game.n_attempt_per_token == game.n_max_attempt_per_token)
          or (game.outcome == -1 and game.n_attempt_per_token == game.n_max_attempt_per_token)):
        entry_log = game.store_info_gen()
        log_gen.add_row_entry(entry_log)

      info = game.store_info_spec(game.outcome)
      log_spec.add_row_entry(info)
      sm.update_counters(game)
      game.reset_counters()
      game.reset_detected_token()

  entry_log = game.store_info_summary()
  log_summary.add_row_entry(entry_log)

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
