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
This pakage generates the facial expressions using OpenCV, it is based on previous work from ... on the Baxter robot
- [MaxEntRL](https://github.com/aandriella/MaxEntRL)
This package generates the robot's policy by the data provided by the GOAL simulator
The IRL code is based on the work of [ Maximilian Luz](https://github.com/qzed/irl-maxent).
- [Generative Mutual Shaping RL](https://github.com/aandriella/GenMutShapRL)
This package generates syntetic data by starting from the BN models of the patient and the robot. The BNs are built by gathering data from real interactions and therapist's expertise on the patient cognitive abilities.
