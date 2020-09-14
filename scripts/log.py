'''This class is used to store all the information concering the game'''
import os
import datetime
import csv
import pandas as pd

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

	def read_csv_file(self, log):
		# making data frame from csv file
		data = pd.read_csv(log)

		# replacing blank spaces with '_'
		#data.columns = [column.replace(" ", "_") for column in data.columns]

		# filtering with query method
		data.query('token_id =="17"  and '
							 'from_=="15"',
							 inplace = True)
		# display
		print(data)



def main():
	variables = ['token_id', 'from', 'to', 'react_time', 'elapsed_time']
	entries_value = {'token_id': 17, 'from': 15, 'to': 2, 'react_time': 1.23, 'elapsed_time': 5.02}
	entries_name = {'token_id': 'token_id', 'from': 'from', 'to': 'to', 'react_time': 'react_time', 'elapsed_time': 'elapsed_time'}

	file = '/home/aandriella/pal/cognitive_game_ws/src/carf/caregiver_in_the_loop/log/csv.txt'
	log = Log(file, fieldnames_spec=variables, fieldnames_gen=variables, fieldnames_sum=variables)
	# log.add_row_entry(entries_name, log.fieldnames_spec)
	# log.add_row_entry(entries_value, log.fieldnames_spec)
	log.read_csv_file(file)

if __name__ == "__main__":
	main()