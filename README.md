# CARESSER: aCtive leARning agEnt aSsiStive bEhaviouR

#### "Introducing CARESSER: a Framework for in Situ Learning Robot Social Assistance from Expert Knowledge and Demonstrations" [Website link](http://www.iri.upc.edu/groups/perception/#CARESSER)

##### Authors: Antonio Andriella, Carme Torras, Carla Abdelnour and Guillem Alenyà


#### Packages:
- [CARESSER (Personalised Robot-Assisted Cognitive Training)](https://github.com/aandriella/CARESSER)
The main package in change of generating the exercise and managing the interactions between the patients and the therapist (either human or robot). It has two branches:
-- caregiver_in_the_loop: the therapist is the human
-- robot_in_the_loop: the therapist is the robot
- [Bayesian Network Generative Model](https://github.com/aandriella/BN_GenerativeModel)
This package generates two bayesian models by combining the data collected from the interactions between the therapist and the patient and the therapist's expertise (from questionnaire).
- [Robot Behaviour](https://github.com/aandriella/robot_behaviour)
This package defines the TIAGo behaviours (SPEECH, GESTURE and FACIAL EXPRESSIONS)
- [Board State](https://github.com/aandriella/board_state)
This package provides the positions of the tokens on the board (RFID + ARDUINO)
- [Robot Face Visalizer](https://github.com/aandriella/robot_face_visualizer)
This package reproduces on a screen some facial expressions 
- [Robot Facial Expression](https://github.com/aandriella/robot_facial_expression)
This pakage generates the facial expressions using OpenCV, it is based on previous work from  Bilgehan NAL  on the Baxter robot ( [git Repo](git@github.com:bilgehannal/baxter_face_software.git))
- [MaxEntRL](https://github.com/aandriella/MaxEntRL)
This package generates the robot's policy by the data provided by the GOAL simulator
The IRL code is based on the work of [ Maximilian Luz](https://github.com/qzed/irl-maxent).
- [Generative mOdel Agent simuLation (GOAL)](https://github.com/aandriella/GOAL)
This package generates syntetic data by using the GOAL simulator. Two BNs are built one which models the therapist (human and robotic) and the other which models the patient.
In implements the simulator which 
The BNs are built by gathering data from real interactions and therapist's expertise on the patient cognitive abilities.

#### Package:
- **launch** contains the launch file which run the entire framework. It contains also all the pre-recorded actions of assistance provided by the robot (only possible to use with the TIAGo)
- **sounds** sounds emitted by the board when a token has been moved. Also timeout and max attempt sounds
- **scripts** it contains:
-- the main class main.py which runs the entire framework and manage the human-robot interactions. 
-- cognitive_game.py defines the type of exercise and manage all the events related to the exercise 
-- rfid_board.py reads from a topic the state of the board and catch any events happening on the board

##### NOTE:
The code is platform-dependent. In order to run this code you need to have a RFID board and a TIAGo robot.


### USAGE

``` 
 roslaunch CARESSER bring_up.launch 
 --language en_GB --config_path "$(path/to/sentences/folder)" --user_id 1 --session_id 1 --objective "descending" --with_feedback True --timeout 15  
```
where :
-- language is the language supported by the TIAGo
-- config_path is the folder with the scripted sentences
-- user_id is the is assigned to the user 
-- session_id is the id of the session objective is the type of exercise (e.g "ascending", "descending", "ascending_odd", etc ...) 
-- with_feedback is if we use [SOCIABLE](http://www.iri.upc.edu/files/scidoc/2353-Discovering-SOCIABLE:-Using-a-conceptual-model-to-evaluate-the-legibility-and-effectiveness-of-backchannel-cues-in-an-entertainment-scenario.pdf) 
-- timeout is the waiting time before providing the user with assistance again if they do not make any move.
