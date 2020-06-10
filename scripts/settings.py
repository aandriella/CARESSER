'''In this class we set all the connecting with the external sources and we
initialise all the classes instances
'''

#import from ros
import rospy
import time
#sub/pub, services and actions from ros
from std_msgs.msg import String, Int8
from iri_rfid_board_scanner.msg import BoardIds

#import from libraries
import pygame

class Settings(object):
	def __init__(self):
		rospy.init_node('big_hero', anonymous=True)
		# subscriber for getting data from the hr sensor
		rospy.Subscriber("/rfid_board_scanner/board_ids", BoardIds, self.get_electro_board_callback)
		self.robot_language = rospy.get_param("/language")
		self.user_id = rospy.get_param("/user_id")
		self.ids = list(map(int, rospy.get_param('/rfid_id').split()))
		self.tokens = rospy.get_param('/tokens_number')
		solution_board_from_launch = rospy.get_param('/solution_board')
		# getting the ids from the rfids sensors
		self.ids = [int(self.ids[i]) for i in range(len(self.ids))]
		# coupling ids with numbers
		self.electro_board_from_ids_to_tokens = self.convert_electro_board_ids_to_tokens(self.ids, self.tokens)
		self.solution_board = solution_board_from_launch
		self.electro_board = list()
		self.electro_board_with_tokens = dict()

		self.previous_board_state = dict()
		self.current_board_state = dict()


	def play_sound(self, file_path, sleep):
		'''
		play a sound using pygame
		'''
		pygame.mixer.init()
		pygame.mixer.music.load(file_path)
		pygame.mixer.music.play(0)
		time.sleep(sleep)


	def get_tokens_id(self):
		return self.tokens


	# def get_robot_personality(self):
	#   return self.robot_personality

	def get_robot_language(self):
		return self.robot_language


	# def get_robot_gender(self):
	#   return self.robot_gender

	def get_user_id(self):
		return self.user_id


	# get value from the topic
	def get_electro_board_ids(self):
		return self.electro_board


	# topic subscriber
	def get_electro_board_callback(self, msg):
		self.electro_board = (msg.id)


	def get_electro_board(self):
		return self.get_tokens_by_ids(self.electro_board_from_ids_to_tokens, self.electro_board)


	def convert_electro_board_ids_to_tokens(self, ids, tokens):
		'''
		:param ids:
		:param tokens:
		:return: this method creates a dic where the ids are the keys and the values are the tokens
		'''
		from_ids_to_tokens = dict()
		for i in range(len(ids)):
			from_ids_to_tokens[ids[i]] = tokens[i]
		return from_ids_to_tokens


	def get_tokens_by_ids(self, from_ids_to_tokens_dict, msg):
		'''
		:param electro_board:
		:param msg:
		:return: given a dictionary where key is the id and value is the token change the msg according (0 empty !0 a token id)
		'''
		for i in range(len(msg)):
			if msg[i] == 0:
				# i+1 because the id start
				self.electro_board_with_tokens[i + 1] = '0'
			else:
				self.electro_board_with_tokens[i + 1] = from_ids_to_tokens_dict.get(msg[i])
		return self.electro_board_with_tokens


def main():
	settings = Settings()
	while not rospy.is_shutdown():
		print(settings.get_electro_board())


if __name__=="__main__":
	main()