Learning Target Tracking with Tabular Q-Lambda or Actor-Critic Method 

#TABULAR Q-LAMBDA

Use runACM=false in main.cpp for running the Q-Lambda algorithm.

The Tabular Q-Lambda Algorithm is just implemented in simulation. The state space consists of the difference of the marker position and the robot arm position in the X and Y direction.The 4 possible actions that can be selected are just moving the position of the robot arm one up or down in y-direction or one left or right in x-direction. 
The different parameters of the algorithm can be set in simulation.h (the parameters set at the moment work fine for a fast run of the algorithm on a 20x20 sized grid).

The simulation class inlcudes: an initialization, an epsilon-greedy action-selection, a state-transition, a reward system, a random movement of the marker (the marker moves on every second iteration of 				       the algorithm), a writing of the Q-Table into a file and the run of the algorithm.

The algorithm runs for 100 iterations at each episode. A plot of the summed reward of this 100 iterations at each episode is shown at the end.


#ACTOR-CRITIC
The features of the actor-critic method are considered the difference of the robot arm position and the marker position. It aims to learn the parameters which are the linear coefficients of the features and apply the learned parameters to select proper actions for target tracking.

The implementation of the actor-critic algorithm contains two parts:
-environment
 including initialization, reward system, state transition and detection of alvar markers

-agent
 including selection of actions, computation of td-error, critic update, actor update and record of learning results

The plots and the end if the simulation is used, shows:  -the number of steps needed to to get to the marker for some test-episodes
							 -the summed reward for this test-episodes
							 -the average reward for each step of the test-episodes (close to 20 would be best,
							  because the reward gets a bonus of 10 if u are near enough in both x-direction and y-direction)




- set runACM in the main.cpp file to "true" if u want to run the Actor-Critic Method and to "false" if u want to run the Tabular Q-Lambda Algorithm.
- set Ros in the main.cpp file to "true" if u want to run on the Baxter (u also have to execute the program with "./x.exe -useRos 1") 
  and to "false" if u want to simulate (for simulation just run the program with "./x.exe").
