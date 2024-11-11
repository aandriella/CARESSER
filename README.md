<div align="center">
<b>Introducing CARESSER: a Framework for in Situ Learning Robot Social Assistance from Expert Knowledge and Demonstrations</b>
<p>
Antonio Andriella <sup>a</sup>, 
Carme Torras <sup>a</sup>, 
Carla Abdelnour <sup>b</sup>, 
Guillem Alenyà <sup>a</sup>
</p>
<p><sup>a</sup> Institut de Robòtica i Informàtica Industrial, CSIC-UPC.</p>
<p><sup>b</sup> Research Center and Memory Clinic, Fundació ACE, Institut Català de Neurociències Aplicades.</p>

  <a href="https://www.iri.upc.edu/groups/perception/CARESSER/">Project webpage</a>
  </br><i>This work has been published on User Modeling and User-Adaptated Interaction Journal</i>
</div>






---

**Abstract:**  
Socially Assistive Robots have the potential to augment and enhance caregivers' effectiveness in repetitive tasks such as cognitive therapies. However, their contribution has generally been limited, as domain experts have not been fully involved in the entire pipeline of the design process, as well as in the automation of robots' behavior. In this article, we present aCtive leARning agEnt aSsiStive bEhaviouR (CARESSER), a novel framework that actively learns robotic assistive behavior by leveraging the therapist's expertise (knowledge-driven approach) and their demonstrations (data-driven approach). By exploiting this hybrid approach, the presented method enables in situ fast learning, in a fully autonomous fashion, of personalized, patient-specific policies. To evaluate our framework, we conducted two user studies in a daily care center in which older adults affected by mild dementia and mild cognitive impairment (N=22) were requested to solve cognitive exercises with the support of a therapist and later of a robot endowed with CARESSER. Results showed that:  
i) the robot was more competent than the therapist in maintaining patients' performance during sessions, and  
ii) the assistance offered by the robot during sessions matched the therapist's preferences. We conclude that CARESSER, with its stakeholder-centric design, can pave the way to new AI approaches that learn by leveraging human-human interactions along with human expertise. This approach benefits the learning process by speeding it up, eliminating the need for complex reward function designs, and avoiding undesired states.


## The CARESSER Framework
<p align="center">
  <img src="CARESSER.jpg" width="350" title="CARESSER FRAMEWORK">
</p>

```diff
- N.B: main branch does not work, need to use either caregiver_in_the_loop or robot_in_the_loop
```

#### Packages:
- [CARESSER (aCtive leARning agEnt aSsiStive bEhaviouR)](https://github.com/aandriella/CARESSER)
The main package in charge of generating the exercise and managing the interactions between the patients and the therapist (either human or robot). It has two branches (ignore the main):
-- caregiver_in_the_loop: the therapist is the human
-- robot_in_the_loop: the therapist is the robot
- [Bayesian Network Generative Model](https://github.com/aandriella/BN_GenerativeModel)
This package generates two bayesian models by combining the data collected from the interactions between the therapist and the patient and the therapist's expertise (from questionnaire). The models are actively refined during the interaction between the robot and the patient.
- [Robot Behaviour](https://github.com/aandriella/robot_behaviour)
This package defines the TIAGo assistive behaviours. We defined 7 different levels of assistance combining speech, gesture and facial expressions.
- [Board State](https://github.com/aandriella/board_state)
This package provides the positions of the tokens on the board (RFID + Arduino).
- [Robot Face Visualizer](https://github.com/aandriella/robot_face_visualizer)
This package reproduces on a screen some facial expressions.
- [Robot Face Expression](https://github.com/aandriella/robot_face_expression)
This pakage generates the facial expressions using OpenCV, it is based on previous work of [Bilgehan NAL](https://github.com/bilgehannal/baxter_face_software) on the Baxter robot.
- [MaxEntIRL](https://github.com/aandriella/MaxEntIRL)
This package generates the robot's policy by the data provided by the GOAL simulator. It is based on the work of [Maximilian Luz](https://github.com/qzed/irl-maxent).
- [GOAL sim](https://github.com/aandriella/GOAL)
This package generates the simulation as well as the policy to embed on the robot by starting from the BN models of the therapist and the robot. The BNs are built by gathering data from real interactions and therapist's expertise on the patient cognitive abilities.
- [Task Env](https://github.com/aandriella/task_environment)
This package contains the cognitive training domain both for the IRL algorithm and for the real time interaction between the patient and the therapist (human or robot).

## Citation

If you use this work, please cite the following publication:

```bibtex
@article{Andriella_UMUAI23,
  title = "Introducing CARESSER: A framework for in situ learning robot social assistance from expert knowledge and demonstrations",
  journal = "User Modeling and User-Adapted Interaction",
  volume = "33",
  pages = "441-496",
  year = "2023",
  doi = "10.1007/s11257-021-09316-5",
  url = "https://doi.org/10.1007/s11257-021-09316-5",
  author = "Antonio Andriella, Carme Torras, Carla Abdelnour, and Guillem Alenyà",
  keywords = {Robot adaptivity, Robot personalisation, Human–robot interaction, Robot-assisted cognitive training, Socially assistive robotics, In situ learning}
}
