#!/usr/bin/python
'''
This is the main class for running the entire game framework
'''
# import modules and classes
from log import Log
from cognitive_game import Game
# import from libraries
import enum
import random
import time
import os
from termcolor import colored
import pygame
# import from ros
import rospy

path = os.path.abspath(__file__)
dir_path = os.path.dirname(path)
parent_dir_of_file = os.path.dirname(dir_path)
parent_parent_dir_of_file = os.path.dirname(parent_dir_of_file)



class StateMachine(enum.Enum):

  def __init__(self, e):
    self.b_caregiver_assist_finished = False
    self.b_caregiver_feedback_finished = False
    self.b_user_picked_token = False
    self.b_user_placed_token_back = False
    self.b_user_placed_token_sol = False
    self.b_caregiver_outcome_finished = False
    self.b_caregiver_moved_token_back = False
    self.b_user_moved_token_back = False
    self.b_caregiver_moved_correct_token = False
    self.b_user_reached_max_attempt = False
    self.b_user_reached_timeout = False
    self.b_caregiver_reengaged_user = False

  '''States of the State machine'''
  S_CAREGIVER_ASSIST = 1
  S_USER_ACTION = 2
  S_USER_PICK_TOKEN = 3
  S_CAREGIVER_FEEDBACK = 4
  S_USER_PLACE = 5
  S_USER_PLACE_TOKEN_BACK = 6
  S_USER_PLACE_TOKEN_SOL = 7
  S_USER_MOVE_TOKEN_BACK = 8
  S_CAREGIVER_MOVE_TOKEN_BACK = 9
  S_CAREGIVER_MOVE_CORRECT_TOKEN = 10
  S_CAREGIVER_OUTCOME = 11
  S_USER_TIMEOUT = 12

  CURRENT_STATE = 1


  def play_sound(self, file_path, sleep):
    '''
    play a sound using pygame
    '''
    pygame.mixer.init()
    # pygame.mixer.music.load(parent_dir_of_file+'/sounds/wrong_move_trim.mp3')
    pygame.mixer.music.load(parent_dir_of_file+"/sounds/"+file_path)
    pygame.mixer.music.play(0)
    time.sleep(sleep)

  def stop_sound(self):
    pygame.mixer.music.stop()

  def caregiver_provide_assistance(self):
    '''
    Caregiver provides an action of assistance combining speech and gesture
    Return:
       True when the action has been completed
    '''
    print("C_ASSISTANCE")
    rospy.sleep(0.1)
    input = raw_input(colored("Please press a key when assistance has been provided",'green'))
    self.b_caregiver_assist_finished = True
    self.b_caregiver_reengaged_user == False
    self.CURRENT_STATE = self.S_USER_ACTION
    return self.b_caregiver_assist_finished

  def update_counters(self, game):
    '''
        Auxiliary function to update the counters based on the outcome
        :param self:
        :return:
        '''
    print("Update_counters")

    if game.moved_back:
      if game.outcome == 0:
        game.n_timeout_per_token += 1

      print("token has been moved back, DO nothing")
      game.n_mistakes += 1
      game.n_attempt_per_token += 1
      game.set_n_attempt_per_token(game.n_attempt_per_token)
      game.set_n_mistakes(game.n_mistakes)

      if game.n_attempt_per_token > game.n_max_attempt_per_token:
        print(colored("Max attempt reached", "red"))
        game.set_n_correct_move(game.get_n_correct_move() + 1)
        game.set_n_attempt_per_token(1)
        self.n_sociable_per_token = 0
        self.n_timeout_per_token = 0



    elif game.outcome == 0:
      game.n_timeout_per_token += 1
      game.n_mistakes += 1
      game.n_attempt_per_token += 1
      game.set_n_attempt_per_token(game.n_attempt_per_token)
      game.set_n_mistakes(game.n_mistakes)

      # check if the user reached his max number of attempts
      if game.n_attempt_per_token > game.n_max_attempt_per_token:
        print(colored("Max attempt reached", "red"))
        game.set_n_correct_move(game.get_n_correct_move() + 1)
        game.set_n_attempt_per_token(1)
        self.n_sociable_per_token = 0
        self.n_timeout_per_token = 0

    # get current move and check if it is the one expeceted in the solution list
    elif game.outcome == 1:
      game.set_n_correct_move(game.get_n_correct_move() + 1)
      game.set_n_attempt_per_token(1)
      self.n_sociable_per_token = 0
      self.n_timeout_per_token = 0

    elif game.outcome == -1:
      print(colored("WRONG SOLUTION", "red"))
      game.n_mistakes += 1
      game.n_attempt_per_token += 1
      game.set_n_attempt_per_token(game.n_attempt_per_token)
      game.set_n_mistakes(game.n_mistakes)

      # check if the user reached his max number of attempts
      if game.n_attempt_per_token > game.n_max_attempt_per_token:
        print(colored("Max attempt reached", "red"))
        game.set_n_correct_move(game.get_n_correct_move() + 1)
        game.set_n_attempt_per_token(1)
        self.n_sociable_per_token = 0
        self.n_timeout_per_token = 0


  def caregiver_provide_outcome(self, game):
    '''
    Caregiver provides the user with the outcome of their move
    :param self:
    :return:
    '''
    print("R_OUTCOME")
    #input = raw_input("Please press a key when outcome has been provided")

    if game.moved_back and not self.b_user_reached_timeout:
      print(colored("token has been moved back", "red"))

      game.outcome = -1
      self.CURRENT_STATE = self.S_CAREGIVER_ASSIST
      # check if the user reached his max number of attempts
      if game.n_attempt_per_token >= game.n_max_attempt_per_token:
        print(colored("Max attempt reached", "red"))
        self.S_CAREGIVER_MOVE_CORRECT_TOKEN = True
        self.b_user_reached_max_attempt = True
        self.play_sound("max_attempt_trim.mp3", 3)
        self.caregiver_move_correct_token(game)

    elif game.detected_token == [] and self.b_user_reached_timeout:
      print(colored("TIMEOUT", 'red'))
      game.outcome = 0
      self.play_sound("timeout_trim.mp3", 3)
      self.CURRENT_STATE = self.S_CAREGIVER_ASSIST
      self.b_user_reached_timeout = False

      if game.n_attempt_per_token >= game.n_max_attempt_per_token:
        print(colored("Max attempt reached", "red"))
        self.S_CAREGIVER_MOVE_CORRECT_TOKEN = True
        self.b_user_reached_max_attempt = True
        self.play_sound("max_attempt_trim.mp3", 3)
        self.caregiver_move_correct_token(game)


    elif game.detected_token != [] and self.b_user_reached_timeout:
      print(colored("TIMEOUT", 'red'))
      game.outcome = 0
      self.play_sound("timeout_trim.mp3", 3)
      self.user_move_back(game)
      self.CURRENT_STATE = self.S_CAREGIVER_ASSIST
      self.b_user_reached_timeout = False

      if not game.check_board():
        print(colored("BOARD HAS TO BE RESET", 'yellow'))
      while not game.check_board():
        print(game.current_board)
        game.current_board = game.get_board_event()
        self.play_sound("something_went_wrong_trim.mp3", 1)


      # check if the user reached his max number of attempts
      if game.n_attempt_per_token >= game.n_max_attempt_per_token:
        print(colored("Max attempt reached", "red"))
        self.S_CAREGIVER_MOVE_CORRECT_TOKEN = True
        self.b_user_reached_max_attempt = True
        self.play_sound("max_attempt_trim.mp3", 3)
        self.caregiver_move_correct_token(game)

    # get current move and check if it is the one expeceted in the solution list
    elif game.detected_token[0] == game.solution[game.n_correct_move] \
        and game.detected_token[2] == (game.solution.index(game.detected_token[0]) + 1):
      game.outcome = 1
      print(colored("CORRECT:", 'red'))
      print(colored(game.get_n_correct_move(), 'red'))
      self.CURRENT_STATE = self.S_CAREGIVER_ASSIST

    elif game.detected_token[0] != game.solution[game.n_correct_move] \
        or game.detected_token[2] != game.solution.index(game.detected_token[0]) + 1:
      game.outcome = -1
      print(colored("WRONG",'red'))
      print(colored(game.n_attempt_per_token, 'red'))
      self.CURRENT_STATE = self.S_CAREGIVER_MOVE_TOKEN_BACK
      self.user_move_back(game)

      # check if the user reached his max number of attempts
      if game.n_attempt_per_token >= game.n_max_attempt_per_token:
        print(colored("Max attempt reached", "red"))
        self.S_CAREGIVER_MOVE_CORRECT_TOKEN = True
        self.b_user_reached_max_attempt = True
        self.caregiver_move_correct_token(game)
        self.play_sound("max_attempt_trim.mp3", 3)

    self.b_caregiver_outcome_finished = True
    return self.b_caregiver_outcome_finished

  def caregiver_move_correct_token(self, game):
    print(colored("Waiting for caregiver to move the correct token ....", 'red'))
    # get the current solution
    token, _from, _to = game.get_token_sol()
    while (game.detected_token != (token, _from, _to)):
      pass
    print(colored("Caregiver moves the correct token as the user reached the max number of attempts", 'red'))
    self.CURRENT_STATE = self.S_CAREGIVER_ASSIST
    self.b_user_moved_token_back = True
    return self.b_caregiver_moved_correct_token

  def caregiver_move_back(self, game):
    # user moved the token in an incorrect location
    # caregiver moved it back
    token_id, token_from = game.detected_token[0],game.detected_token[1]
    token_to = game.initial_board[game.initial_board.values(token_id)]
    while(token_from != token_to):
      pass
    print(colored("Caregiver moves back the token ...", 'red'))
    self.CURRENT_STATE = self.S_CAREGIVER_ASSIST
    self.b_caregiver_moved_token_back = True
    return self.b_caregiver_moved_token_back

  def user_move_back(self, game):
    # user moved the token in an incorrect location
    # caregiver moved it back
    print(colored("User moves back the token ...", 'red'))
    #success = caregiver.action["move_back"].__call__(who="user", token=game.get_token_sol(), counter=game.n_attempt_per_token-1a)
    # get the initial location of the placed token and move back there
    token_id, token_from, token_to = game.detected_token
    if token_to == 0:
      #means you have the token in your hand
      token_to = token_from
    curr_token, _, curr_token_to = game.detected_token
    while (curr_token== (token_id) and curr_token_to != token_from):
      curr_token, _, curr_token_to = game.detected_token
      pass

    self.CURRENT_STATE = self.S_CAREGIVER_ASSIST
    self.b_user_moved_token_back = True
    return self.b_user_moved_token_back

  def user_action(self, game):
    '''user action state has been divided in three sub methods:
     1a. user pick a token
     1a.a check for timeout
        1a.b timeout -> provide reengagement
        go back to 1a
     2. caregiver provides feedback
     3. user places a token
     3.a user places a token back
     3.b user places a token in a goal location
      '''
    print("U_ACTION")

    def check_move_timeout(game, time_to_start):
      '''we need to check if the computed time (react+elapsed) is smaller than timeout
      if not, we need to trigger different actions:
      1a. if the user has not picked a token (re-engage)
      2. if the user has picked a token, ask her to move it back
      '''
      current_time = time.time()
      elapsed_time = current_time - time_to_start
      if elapsed_time < game.timeout:
        return True
      else:
        return False

    def user_pick_token(sm, game):
      game.react_time_per_token_spec_t0 = time.time()
      print("U_PICK")
      sm.CURRENT_STATE = sm.S_CAREGIVER_FEEDBACK
      detected_token, picked, _, _ = game.get_move_event()
      while (not picked and (detected_token == [])):
        if check_move_timeout(game, game.react_time_per_token_spec_t0):
          detected_token, picked, _, moved_back = game.get_move_event()
        else:
          sm.b_user_reached_timeout = True
          game.react_time_per_token_spec_t1 = game.timeout
          game.elapsed_time_per_token_spec_t1 = game.timeout
          return False

      game.elapsed_time_per_token_spec_t0 = time.time()
      game.react_time_per_token_spec_t1 = time.time() - game.react_time_per_token_spec_t0
      game.react_time_per_token_gen_t1 += game.react_time_per_token_spec_t1
      sm.b_user_picked_token = True
      return sm.b_user_picked_token

    def caregiver_provide_feedback(sm):
      print("C_FEEDBACK")
      game.n_sociable_per_token += 1
      game.n_tot_sociable += 1
      sm.CURRENT_STATE = sm.S_USER_PLACE
      sm.caregiver_provided_feeback_finished = True
      return sm.caregiver_provided_feeback_finished

    ''' When the user picked the token we check where they place it'''
    '''either they can place it back '''

    def user_place(sm, game):
      '''user place has been divided into:
         1a. user places a token back
         2. user places a token in a solution row
      '''
      print("U_PLACE")
      # check where the user will place the token
      def user_place_token_back(sm):
        print("U_PLACE_BACK")
        sm.CURRENT_STATE = sm.S_CAREGIVER_OUTCOME
        sm.b_user_placed_token_back = True
        game.elapsed_time_per_token_spec_t1 = time.time() - game.elapsed_time_per_token_spec_t0
        game.elapsed_time_per_token_gen_t1 += game.elapsed_time_per_token_spec_t1
        return sm.b_user_placed_token_back

      '''or they can place it in the solution row'''

      def user_place_token_sol(sm):
        print("U_PLACE_SOL")
        sm.CURRENT_STATE = sm.S_CAREGIVER_OUTCOME
        sm.b_user_placed_token_sol = True
        game.elapsed_time_per_token_spec_t1 = time.time() - game.elapsed_time_per_token_spec_t0
        game.elapsed_time_per_token_gen_t1 += game.elapsed_time_per_token_spec_t1
        return sm.b_user_placed_token_sol

      '''return where the user placed the token'''

      # here we check whether a token has been picked, and where it has been placed
      detected_token, picked, placed, moved_back = game.get_move_event()
      while (not placed):
        if check_move_timeout(game, game.elapsed_time_per_token_spec_t0):
          detected_token, _, placed, moved_back = game.get_move_event()
        else:
          sm.CURRENT_STATE = sm.S_CAREGIVER_OUTCOME
          break
      if (placed and moved_back):
        sm.CURRENT_STATE = sm.S_CAREGIVER_OUTCOME
        return user_place_token_back(sm)
      elif (placed and not moved_back):
        sm.CURRENT_STATE = sm.S_CAREGIVER_OUTCOME
        return user_place_token_sol(sm)
      else:
        sm.b_user_reached_timeout = True
        game.elapsed_time_per_token_spec_t1 = game.timeout
        game.react_time_per_token_spec_t1 = game.timeout
        return False

    self.CURRENT_STATE = self.S_USER_ACTION
    # if the user picks a token and SOCIABLE is active
    if user_pick_token(self, game):
      if user_place(self, game):
        return True
      else:
        #TIMEOUT
        self.CURRENT_STATE = self.S_CAREGIVER_OUTCOME
    else:
      print("user_pick_token==False")
      self.CURRENT_STATE = self.S_CAREGIVER_OUTCOME

  def num_to_func_to_str(self, argument):
    switcher = {
      0: self.caregiver_provide_assistance,
      1: self.caregiver_reengage_user,
      2: self.caregiver_provide_feedback,
      3: self.user_action,
      4: self.caregiver_provide_outcome,
      5: self.caregiver_move_back,
      6: self.user_move_back,
      7: self.caregiver_move_correct_token
    }

    # get the function based on argument
    func = switcher.get(argument)

    # Execute the function
    return func()


def main():
  user_id = rospy.get_param("/user_id")
  session_id = rospy.get_param("/session_id")
  with_feedback = rospy.get_param("/with_feedback")
  objective = rospy.get_param("/objective")
  timeout = rospy.get_param("/timeout")

  # we create the game instance
  game = Game(board_size=(5, 4), task_length=5, n_max_attempt_per_token=4,
              timeout=timeout, objective=objective,
              bn_game_state={'beg': 2, 'mid': 4, 'end': 5}, bn_attempt={'att_1':0, 'att_2':1, 'att_3':2, 'att_4':3},
              bn_caregiver_feedback={'no':0,'yes':1},
              bn_caregiver_assistance={'lev_0':0,'lev_1':1,'lev_2':2,'lev_3':3,'lev_4':4,'lev_5':5},
              bn_user_react_time={'fast': 5, 'normal': 10, 'slow': 15},
              bn_user_action={'correct': 0, 'wrong': 1, 'timeout': 2})

  #user_id = raw_input("please, insert the id of the user:")
  path = os.path.abspath(__file__)
  dir_path = os.path.dirname(path)
  parent_dir_of_file = os.path.dirname(dir_path)
  path_name = parent_dir_of_file + "/caregiver_in_the_loop/log/" + str(user_id)+"/"+str(with_feedback)+"/"+str(session_id)

  try:
      os.makedirs(path_name)
  except IOError:
      print("Error the directory already exists")
      exit(0)


  print("You are playing with the following numbers: ", game.current_board)
  print("The solution of the game is ", game.solution)
  input = raw_input("Press a key to start:")

  file_spec = path_name + "/log_spec.csv"
  file_gen = path_name + "/log_gen.csv"
  file_summary = path_name + "/log_summary.csv"
  file_bn_variables = path_name+"/bn_variables.csv"
  bn_belief_user_action_file = path_name + "/bn_belief_user_action.pkl"
  bn_belief_user_react_time_file = path_name + "/bn_belief_user_react_time.pkl"
  bn_belief_caregiver_assistive_action = path_name + "/bn_belief_caregiver_assistive_action.pkl"
  bn_belief_caregiver_feedback_action = path_name + "/bn_belief_caregiver_feedback_action.pkl"

  entry_log_spec = {'game_state':'game_state', 'user_react_time':'user_react_time', 'user_action':'user_action', 'token_id':'token_id', 'from':'from', 'to':'to',
                    'caregiver_assistance':'caregiver_assistance', "react_time":'react_time',
                    'elapsed_time':'elapsed_time', "attempt":'attempt', "timeout":'timeout', "caregiver_feedback":'caregiver_feedback'}

  entry_log_gen = {"token_id":'token_id', 'user_action':'user_action', "from":'from', "to":'to', "avg_caregiver_assistance_per_move":'avg_caregiver_assistance_per_move',
                  "cum_react_time":"cum_react_time", "cum_elapsed_time":"cum_elapsed_time", "attempt":"attempt",
                  "timeout":"timeout", "caregiver_feedback":"caregiver_feedback"}
  entry_log_summary = {"n_attempt":"n_attempt", "n_timeout":"n_timeout", "n_sociable":"n_sociable",
                       "avg_lev_assistance":"avg_lev_assistance", "tot_react_time":"tot_react_time",
                       "tot_elapsed_time":"tot_elapsed_time"}
  entry_bn_variables = {"game_state":"game_state","attempt":"attempt", "user_react_time":"user_react_time",
                        "agent_assistance":"agent_assistance", "agent_feedback":"agent_feedback",
                        "user_action":"user_action", "user_memory":"user_memory", "user_reactivity":"user_reactivity"}


  log = Log(filename_spec=file_spec, fieldnames_spec=entry_log_spec, filename_gen=file_gen,
            fieldnames_gen=entry_log_gen, filename_sum=file_summary, fieldnames_sum=entry_log_summary,
            filename_bn_vars= file_bn_variables, fieldnames_bn_vars=entry_bn_variables
            )

  log.add_row_entry(log_filename=file_spec, fieldnames=entry_log_spec, data=entry_log_spec)
  log.add_row_entry(log_filename=file_gen, fieldnames=entry_log_gen, data=entry_log_gen)
  log.add_row_entry(log_filename=file_summary, fieldnames=entry_log_summary, data=entry_log_summary)
  log.add_row_entry(log_filename=file_bn_variables, fieldnames=entry_bn_variables, data=entry_bn_variables)

  sm = StateMachine(1)


  while game.get_n_correct_move() < game.task_length:

    if sm.CURRENT_STATE.value == sm.S_CAREGIVER_ASSIST.value:
      rospy.sleep(0.1)
      while not game.check_board():

        print(game.current_board)
        game.current_board = game.get_board_event()
        sm.play_sound("something_went_wrong_trim.mp3", 1)

      sm.caregiver_provide_assistance()
      game.avg_caregiver_assistance_per_move += game.caregiver_assistance

    elif sm.CURRENT_STATE.value == sm.S_USER_ACTION.value:
      while not game.check_board() and not sm.b_user_reached_timeout:
        print(game.current_board)
        sm.play_sound("something_went_wrong_trim.mp3", 1)

      print("Expected token ", game.solution[game.get_n_correct_move()])
      time_to_act = time.time()
      #game.with_SOCIABLE = random.randint(0,1a)
      sm.user_action(game)
      game.total_elapsed_time += time.time() - time_to_act

    elif sm.CURRENT_STATE.value == sm.S_CAREGIVER_OUTCOME.value:
      # these are reported only because the variables are already reset when a correct move occurred
      sm.caregiver_provide_outcome(game)

      if (game.outcome == 1 or (game.outcome == 0 and game.n_attempt_per_token == game.n_max_attempt_per_token)
          or (game.outcome == -1 and game.n_attempt_per_token == game.n_max_attempt_per_token)):
        entry_log = game.store_info_gen()
        log.add_row_entry(log_filename=file_gen, fieldnames=entry_log_gen, data=entry_log)
        game.reset_counters_gen()

      entry_log = game.store_info_spec(game.outcome)
      log.add_row_entry(log_filename=file_spec, fieldnames=entry_log_spec, data=entry_log)
      log.add_row_entry(log_filename=file_bn_variables, fieldnames=entry_bn_variables, data=game.store_bn_variables(game.outcome))
      sm.update_counters(game)
      game.reset_counters_spec()
      game.reset_detected_token()


  entry_log = game.store_info_summary()
  log.add_row_entry(log_filename=file_summary, fieldnames=entry_log_summary, data=entry_log)
  log.save_bn_matrix(file_name=bn_belief_user_action_file, bn_dict_vars=game.bn_dict_vars_user_action)
  log.save_bn_matrix(file_name=bn_belief_user_react_time_file, bn_dict_vars=game.bn_dict_vars_user_action)
  log.save_bn_matrix(file_name=bn_belief_caregiver_assistive_action, bn_dict_vars=game.bn_dict_vars_caregiver_assistance)
  log.save_bn_matrix(file_name=bn_belief_caregiver_feedback_action, bn_dict_vars=game.bn_dict_vars_caregiver_feedback)

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
