How to make Baxter write:

	-Put something to write in the gripper (preferable a marker); 
	try to fix it to the gripper;
	-Set the table under the starting position of the endeffector;
	-The arm will go down until it "feels" the table; 
	because the force measurement is noisy, there is a chance the arm will stop before touching the table;
	-After Baxter reaches the table, the simulation of the trajectory starts;
	-When the simulation is finished, Baxter will start to write.



 *The file "qStatesWriteBaxter.txt" contains the trajectory for writing "BAXTER", simulated in position control. For this to work, the table should be set exactly at 73 cm, with a marker in the gripper. ( the starting position for the endeffector is ARR(0.6, -0.1, 0.965) )

 *To simulate the movement for writing something in position control, the function 
- void writeLettersToFile(CtrlTask *pos, arr &y, arr delta_y, arr nrSteps, arr force, ofstream &myfile);

should be used instead of 

- void writeLetterSim(ofstream& myfile, arr delta_y, arr nrSteps, arr force, CtrlTask &pos_eeL, arr q0);

that simulates the writing using inverse kinematics.

A video of Baxter writing using configurations simulated in position control can be found at:
https://youtu.be/c_P74Go2uUA
	
