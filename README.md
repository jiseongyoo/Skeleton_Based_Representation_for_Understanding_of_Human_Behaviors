Class: CSCI573 Human-Centered Robotics at Colorado School of Mines
Lecturer: Dr. Hao Zhang

Author: Jiseong Yoo
Date: 3/16/2018

Program Description:
 This program creates skeleton based representations of human behaviors; RAD (Relative Angles and Distances), HJPD (Histogram of Joint Position Differences), HOD (Histogram of Oriented Displacements).
 
![alt text](https://i1.wp.com/s3.amazonaws.com/production-wordpress-assets/blog/wp-content/uploads/2016/11/23155646/RAD.png?fit=500%2C270&ssl=1)

 Using a library for Support Vector Machine (LIBSVM), the program is trained to classify human behaviors with a dataset collected from Microsoft Kinect V1 sensor.
 This program also predicts human behaviors using the Support Vector Machine.
 The overall accuracy is found in the directory, Accuracy tabel.
 
![alt text](/AccuracyTable/accuracytable.jpg? raw=true "Accuracy Table") 

 Link to the LIBSVM webpage:
	http://www.csie.ntu.edu.tw/~cjlin/libsvm

=> How to compile the .cpp
1. In a new terminal
	$ cd ~/T2_yoo_jiseong
	$ catkin_make

=> How to run the code
1. In a new terminal:
	$ roscore
2. In a new terminal:
	$ cd ~/T2_yoo_jiseong
	$ source ./devel/setup.bash
	$ rosrun p2_t2_a p2_t2_a
	$ rosrun p2_t2_b p2_t2_b
	$ rosrun p2_t2_c p2_t2_c

=> Accuracy of SVM training
1. RAD  : 70.4545%
	The number of bins : 8
2. HJPD : 64.5833%
	The number of bins : 10
3. HOD  : 91.6667%
	The number of bins : 6

=> Accuracy table is included in "Accuracy table"

=> Best values of C and gamma
1. RAD  : C = 128, gamma = 0.03125
2. HJPD : C = 128, gamma = 0.001953125
3. HOD  : C = 2,   gamma = 0.001953125

=> Graphs of the grid search is included in "Graphs of the grid search"
