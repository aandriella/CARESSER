'''This class is used to store all the information concering the game'''
import os
import datetime

class Log():
	def __init__(self, filename, **argv):
		self.filename = filename
		self.variables = self.add_variables(**argv)

	def add_variables(self, **argv):
		with open(self.filename, 'a') as f:
			for arg in argv:
				f.write(arg+"\t")
			f.write("\n")
		f.close()

	def add_row_entry(self, **argv):
		with open(self.filename, 'a') as f:
			for arg in argv:
				f.write(argv.get(arg)+"\t")
			f.write("\n")
		f.close()



def main():
	variables = {'token_id':'', 'from':'', 'to':'', 'react_time':'', 'elapsed_time':''}
	entries = {'token_id': '17', 'from': '15', 'to': '2', 'react_time': '1.23', 'elapsed_time': '5.02'}

	# Log files
	now = datetime.datetime.now()
	folder_name = str(now.year) + str(now.month) + str(now.day) + str(now.hour) + str(now.minute)
	path_name = os.getcwd() +"/log/"+folder_name
	if not os.path.exists(path_name):
		os.makedirs(path_name)


	file = path_name+"/log.txt"
	log = Log(file, **variables)

	log.add_row_entry(**entries)



if __name__ == "__main__":
	main()