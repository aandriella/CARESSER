'''This class is used to store all the information concering the game'''
import os
import datetime
import csv
import pandas as pd
import pickle


class Log():
  def __init__(self, filename_spec, fieldnames_spec, filename_gen, fieldnames_gen, filename_sum, fieldnames_sum):
    self.filename_spec = filename_spec
    self.filename_gen = filename_gen
    self.filename_sum = filename_sum
    self.fieldnames_spec = fieldnames_spec
    self.fieldnames_gen = fieldnames_gen
    self.fieldnames_sum = fieldnames_sum

  def add_row_entry(self, log_filename, fieldnames, data):
    with open(log_filename, mode='a') as csv_file:
      writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
      writer.writerow(data)
    csv_file.close()

  def save_bn_matrix(self, file_name,  bn_dict_vars):

    # Saving the objects:
    with open(file_name, 'wb') as handle:  # Python 3: open(..., 'wb')
      pickle.dump(bn_dict_vars, handle, protocol=pickle.HIGHEST_PROTOCOL)

  def load_bn_matrix(self, file_name):
    # Getting back the objects:
    with open(file_name, 'rb') as handle:
      bn_dict_vars = pickle.load(handle)

    return bn_dict_vars

  def query_csv_file(self, bn_matrix_filename, filename, n_game_state, n_attempt, n_feedback, n_assistance, n_react_time, n_user_action):
    data = pd.read_csv(filename)

    caregiver_feedback_per_action = [[0 for i in range(n_feedback)] for j in
                                          range(n_user_action)]
    caregiver_assistance_per_action = [[0 for i in range(n_assistance)] for j in
                                           range(n_user_action)]
    caregiver_feedback_per_react_time = [[0 for i in range(n_feedback)] for j in
                                              range(n_react_time)]
    caregiver_assistance_per_react_time = [[0 for i in range(n_assistance)] for j in
                                                range(n_react_time)]
    game_state_counter_per_caregiver_assistance = [[0 for i in range(n_game_state)] for j in
                                                        range(n_assistance)]
    attempt_counter_per_caregiver_assistance = [[0 for i in range(n_attempt)] for j in
                                                     range(n_assistance)]
    game_state_counter_per_caregiver_feedback = [[0 for i in range(n_game_state)] for j in
                                                      range(n_feedback)]
    attempt_counter_per_caregiver_feedback = [[0 for i in range(n_attempt)] for j in
                                                   range(n_feedback)]

    #game_state given assistance
    for i in range(n_game_state):
      query_game_state = (data[data.game_state == i].head())
      for j in range(len(query_game_state.caregiver_assistance.values)):
        val = (query_game_state.caregiver_assistance.values[j])
        game_state_counter_per_caregiver_assistance[val][i] += 1
    #attempt given assistance
    for i in range(n_attempt):
      query_attempt = (data[data.attempt==i+1].head())
      for j in range(len(query_attempt.caregiver_assistance.values)):
        val = (query_attempt.caregiver_assistance.values[j])
        attempt_counter_per_caregiver_assistance[val][i] += 1
    #game state given feedback
    for i in range(n_game_state):
      query_game_state = (data[data.game_state == i].head())
      for j in range(len(query_game_state.caregiver_feedback.values)):
        val = (query_game_state.caregiver_feedback.values[j])
        game_state_counter_per_caregiver_feedback[val][i] += 1
    #attempt given feedback
    for i in range(n_attempt):
      query_attempt = (data[data.attempt == i+1].head())
      for j in range(len(query_attempt.caregiver_feedback.values)):
        val = (query_attempt.caregiver_feedback.values[j])
        attempt_counter_per_caregiver_feedback[val][i] += 1

    for i in range(n_assistance):
      query_assistance = (data[data.caregiver_assistance == i].head())
      for j in range(len(query_assistance.user_action)):
        val = (query_assistance.user_action.values[j])
        caregiver_assistance_per_action[val][i] += 1
    for i in range(n_feedback):
      query_feedback = (data[data.caregiver_feedback == i].head())
      for j in range(len(query_feedback.user_action)):
        val = (query_feedback.user_action.values[j])
        caregiver_feedback_per_action[val][i] += 1

    for i in range(n_assistance):
      query_assistance = (data[data.caregiver_assistance == i].head())
      for j in range(len(query_assistance.user_react_time)):
        val = (query_assistance.user_react_time.values[j])
        caregiver_assistance_per_react_time[val][i] += 1
    for i in range(n_feedback):
      query_feedback = (data[data.caregiver_feedback == i].head())
      for j in range(len(query_feedback.user_react_time)):
        val = (query_feedback.user_react_time.values[j])
        caregiver_feedback_per_react_time[val][i] += 1

    bn_matrix = self.load_bn_matrix(bn_matrix_filename)
    bn_matrix['caregiver_feedback_per_action'] = caregiver_feedback_per_action
    bn_matrix['caregiver_assistance_per_action'] = caregiver_assistance_per_action
    bn_matrix['caregiver_feedback_per_react_time'] = caregiver_feedback_per_react_time
    bn_matrix['caregiver_assistance_per_react_time'] = caregiver_assistance_per_react_time
    bn_matrix['game_state_counter_per_caregiver_assistance'] = game_state_counter_per_caregiver_assistance
    bn_matrix['game_state_counter_per_caregiver_feedback'] = game_state_counter_per_caregiver_feedback
    bn_matrix['attempt_counter_per_caregiver_assistance'] = attempt_counter_per_caregiver_assistance
    bn_matrix['attempt_counter_per_caregiver_feedback'] = attempt_counter_per_caregiver_feedback

    print("Good job")
    return bn_matrix




def main():
  entry_log_spec = {'game_state': 'game_state', 'token_id': 'token_id', 'from': 'from', 'to': 'to',
                    'caregiver_assistance': 'caregiver_assistance', "react_time": 'react_time',
                    'elapsed_time': 'elapsed_time', "attempt": 'attempt', "timeout": 'timeout', "sociable": 'sociable'}
  entry_log_gen = {"token_id": 'token_id', "from": 'from', "to": 'to',
                   "avg_caregiver_assistance_per_move": 'avg_caregiver_assistance_per_move',
                   "cum_react_time": "cum_react_time", "cum_elapsed_time": "cum_elapsed_time", "attempt": "attempt",
                   "timeout": "timeout", "sociable": "sociable"}
  entry_log_summary = {"n_attempt": "n_attempt", "n_timeout": "n_timeout", "n_sociable": "n_sociable",
                       "avg_lev_assistance": "avg_lev_assistance", "tot_react_time": "tot_react_time",
                       "tot_elapsed_time": "tot_elapsed_time"}

  file_spec = "" + "/log_spec.csv"
  file_gen = "" + "/log_gen.csv"
  file_summary = "" + "/log_summary.csv"
  log = Log(filename_spec=file_spec, fieldnames_spec=entry_log_spec, filename_gen=file_gen,
            fieldnames_gen=entry_log_gen, filename_sum=file_summary, fieldnames_sum=entry_log_summary)
  # # log.add_row_entry(entries_name, log.fieldnames_spec)
  # # log.add_row_entry(entries_value, log.fieldnames_spec)
  # log.read_csv_file(file)
  # attempt_counter_per_action = [0][0]
  # game_state_counter_per_action = [0][0]
  # caregiver_feedback_per_action = [0][0]
  # caregiver_assistance_per_action = [0][0]
  # attempt_counter_per_react_time = [0][0]
  # game_state_counter_per_react_time = [0][0]
  # robot_feedback_per_react_time = [0][0]
  # robot_assistance_per_react_time = [0][0]
  # game_state_counter_per_robot_assistance = [0][0]
  # attempt_counter_per_robot_assistance = [0][0]
  # game_state_counter_per_robot_feedback = [0][0]
  # attempt_counter_per_robot_feedback = [0][0]
  # bn_dict_vars = {'attempt_counter_per_action':attempt_counter_per_action,
  # 								'game_state_counter_per_action':game_state_counter_per_action,
  # 								'caregiver_feedback_per_action':caregiver_feedback_per_action,
  # 								'caregiver_assistance_per_action':caregiver_assistance_per_action,
  # 								'attempt_counter_per_react_time':attempt_counter_per_react_time,
  # 								'game_state_counter_per_react_time':game_state_counter_per_react_time,
  # 							 	'robot_feedback_per_react_time':robot_feedback_per_react_time,
  # 								'robot_assistance_per_react_time':robot_assistance_per_react_time,
  # 								'game_state_counter_per_robot_assistance':game_state_counter_per_robot_assistance,
  # 								'attempt_counter_per_robot_assistance':attempt_counter_per_robot_assistance,
  # 								'game_state_counter_per_robot_feedback':game_state_counter_per_robot_feedback,
  # 								'attempt_counter_per_robot_feedback':attempt_counter_per_robot_feedback
  # 		}
  #
  # #log.save_bn_matrix("/home/aandriella/pal/cognitive_game_ws/src/carf/caregiver_in_the_loop/log/12test", bn_dict_vars)

  # bn_dict_vars = log.load_bn_matrix("/home/aandriella/pal/cognitive_game_ws/src/carf/caregiver_in_the_loop/log/2/bn_matrix.pkl")
  # print(bn_dict_vars)
  filename = "/home/aandriella/pal/cognitive_game_ws/src/carf/caregiver_in_the_loop/log/3/log_spec.csv"
  bn_filename = "/home/aandriella/pal/cognitive_game_ws/src/carf/caregiver_in_the_loop/log/0/bn_matrix.pkl"
  bn_matrix = log.query_csv_file(bn_matrix_filename=bn_filename, filename=filename, n_game_state=3, n_attempt=4, n_feedback=2, n_assistance=6, n_react_time=3, n_user_action=3)
  print(bn_matrix)


if __name__ == "__main__":
  main()