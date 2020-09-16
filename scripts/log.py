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
		with open(file_name+'.pkl', 'wb') as handle:  # Python 3: open(..., 'wb')
			pickle.dump(bn_dict_vars, handle, protocol=pickle.HIGHEST_PROTOCOL)

	def load_bn_matrix(self, file_name):
		# Getting back the objects:
		with open(file_name+'.pkl', 'rb') as handle:
			bn_dict_vars = pickle.load(handle)

		return bn_dict_vars




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
	file = '/home/aandriella/pal/cognitive_game_ws/src/carf/caregiver_in_the_loop/log/csv.txt'
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

	bn_dict_vars = log.load_bn_matrix("/home/aandriella/pal/cognitive_game_ws/src/carf/caregiver_in_the_loop/log/2/bn_matrix.pkl")

	print(bn_dict_vars)

if __name__ == "__main__":
	main()