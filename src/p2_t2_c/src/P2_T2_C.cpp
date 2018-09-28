#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include <string>
#include <algorithm>
#include <malloc.h>
#include "scale.h"
#include "train.h"
#include "predict.h"

#define NO_JOINT 20		// The number of joints in a skeleton
#define BUFFER_SIZE 100
#define BIN_ANGLE 6	// The number of bins for each angle

//using namespace std

struct Joint_info{
	unsigned int JointNumber;
	float x;
	float y;
	float z;
};

struct Frame_info{
	unsigned int Frame_ID;
	float xy[NO_JOINT][2];
	float yz[NO_JOINT][2];
	float xz[NO_JOINT][2];
};

struct MagAng_info{
	unsigned int Time_ID;
	float mag[NO_JOINT][3];		// magnitude xy, yz, xz
	float ang[NO_JOINT][3];		// angle xy, yz, xz
};

Frame_info* frameDataResize(Frame_info source[],unsigned int add_size);
float getMagnitude(float a, float b);
float getAngle(float a, float b);
void computeMagAng(MagAng_info magangData[], unsigned int t, Frame_info frameData[]);
void createHistogram(float histogramHOD[NO_JOINT][3][BIN_ANGLE], unsigned int start, unsigned int end, MagAng_info magangData[]);
void normalizeHistogram(float histogramHOD[NO_JOINT][3][BIN_ANGLE]);
void concatenateHistogram(float histogramHOD[7*(NO_JOINT)*3*(BIN_ANGLE)], float histogramHOD_11[NO_JOINT][3][BIN_ANGLE], float histogramHOD_21[NO_JOINT][3][BIN_ANGLE], float histogramHOD_22[NO_JOINT][3][BIN_ANGLE], float histogramHOD_31[NO_JOINT][3][BIN_ANGLE], float histogramHOD_32[NO_JOINT][3][BIN_ANGLE], float histogramHOD_33[NO_JOINT][3][BIN_ANGLE], float histogramHOD_34[NO_JOINT][3][BIN_ANGLE]);

int main(int argc, char** argv) {

	// Initialize key variables
	unsigned int frame_size = BUFFER_SIZE;
	Frame_info* frameData = new Frame_info[frame_size]();
	Joint_info joint[NO_JOINT];		// joint[0] = joint1, joint[1] = joint2, joint[2] = joint3, joint[3] = joint4, ...
	
	// input file path
	std::string filePath[] = {"./dataset/train/a00_s00_e00_skeleton_proj.txt", "./dataset/test/a00_s00_e00_skeleton_proj.txt"};
	const std::string activity[] = {"01","02","03","04","05","06","07","08","09","10","11","12","13","14","15","16"};
	const std::string subject[] = {"01", "02", "03", "04", "05", "06", "07", "08", "09", "10"};
	const std::string trial[] = {"01","02"};

	// output file
	std::ofstream createFileTrain("hod_d2");
	std::ofstream createFileTest("hod_d2.t");
	
	for (int TrainTest = 0; TrainTest < 2; TrainTest++)
	{
		// activity
		for(unsigned int k=0;k<16;k++)
		{
			// subject
			for(unsigned int j=0;j<10;j++)
			{
				// trial
				unsigned int frameNumber = 1;
				for(unsigned int i=0;i<2;i++)
				{
					if(TrainTest == 0)	// if training?
					{
						filePath[TrainTest].replace(17,2,activity[k]);
						filePath[TrainTest].replace(21,2,subject[j]);
						filePath[TrainTest].replace(25,2,trial[i]);
					}
					else if(TrainTest == 1)		// if testing?
					{
						filePath[TrainTest].replace(16,2,activity[k]);
						filePath[TrainTest].replace(20,2,subject[j]);
						filePath[TrainTest].replace(24,2,trial[i]);
					}
			
					std::ifstream openFile(filePath[TrainTest].data());
				
					if(openFile.is_open())
					{
						if(TrainTest == 0)
							createFileTrain<<activity[k];
						else if (TrainTest == 1)
							createFileTest<<activity[k];

						while(!openFile.eof())
						{
							if(frameNumber == frame_size)
							{
								// add more buffer (FRAME_BUFFER_SIZE)
								frameData = frameDataResize(frameData, BUFFER_SIZE);
								frame_size = frame_size + BUFFER_SIZE;
							}
						
							for(unsigned int i=0;i<NO_JOINT;i++)
							{
								openFile >> frameNumber;
								openFile >> joint[i].JointNumber;
								openFile >> joint[i].x;
								openFile >> joint[i].y;
								openFile >> joint[i].z;
							}
						
							frameData[frameNumber-1].Frame_ID = frameNumber;
						
							// project 3D to 2D
							for(unsigned int i=0; i<NO_JOINT; i++)
							{
								frameData[frameNumber-1].xy[i][0] = joint[i].x;	frameData[frameNumber-1].xy[i][1] = joint[i].y;
								frameData[frameNumber-1].yz[i][0] = joint[i].y;	frameData[frameNumber-1].yz[i][1] = joint[i].z;
								frameData[frameNumber-1].xz[i][0] = joint[i].x;	frameData[frameNumber-1].xz[i][1] = joint[i].y;
							}
						}
						openFile.close();					
					
						// compute magnitude and angle data
						MagAng_info magangData[frameNumber-1];
						computeMagAng(magangData, frameNumber, frameData);
					
						// create histogram
						unsigned int NoTimeStep = magangData[frameNumber-2].Time_ID;	// The number of time steps
						float histogramHOD_11[NO_JOINT][3][BIN_ANGLE] = {0.0};		// 1-level histogram :  Joint, (xy,yz,xz), Bin
						createHistogram(histogramHOD_11, 0, NoTimeStep-1, magangData);
					
						unsigned int division = NoTimeStep/2;
						float histogramHOD_21[NO_JOINT][3][BIN_ANGLE] = {0.0};		// 2-level 1st histogram
						float histogramHOD_22[NO_JOINT][3][BIN_ANGLE] = {0.0};		// 2-level 2nd histogram
						createHistogram(histogramHOD_21, 0, division-1, magangData);
						createHistogram(histogramHOD_22, division, NoTimeStep-1, magangData);
					
						division = NoTimeStep/4;
						float histogramHOD_31[NO_JOINT][3][BIN_ANGLE] = {0.0};		// 3-level 1st histogram
						float histogramHOD_32[NO_JOINT][3][BIN_ANGLE] = {0.0};		// 3-level 2nd histogram
						float histogramHOD_33[NO_JOINT][3][BIN_ANGLE] = {0.0};		// 3-level 3rd histogram
						float histogramHOD_34[NO_JOINT][3][BIN_ANGLE] = {0.0};		// 3-level 4th histogram
						createHistogram(histogramHOD_31, 0, division-1, magangData);
						createHistogram(histogramHOD_32, division, 2*division-1, magangData);
						createHistogram(histogramHOD_33, 2*division, 3*division-1, magangData);
						createHistogram(histogramHOD_34, 3*division, NoTimeStep-1, magangData);
					
						// normalization
						normalizeHistogram(histogramHOD_11);
						normalizeHistogram(histogramHOD_21);
						normalizeHistogram(histogramHOD_22);
						normalizeHistogram(histogramHOD_31);
						normalizeHistogram(histogramHOD_32);
						normalizeHistogram(histogramHOD_33);
						normalizeHistogram(histogramHOD_34);
					
						// concatenate histogram
						float histogramHOD[7*(NO_JOINT)*3*(BIN_ANGLE)] = {0.0};
						concatenateHistogram(histogramHOD, histogramHOD_11, histogramHOD_21, histogramHOD_22, histogramHOD_31, histogramHOD_32, histogramHOD_33, histogramHOD_34);
					
						// create a single line histogram
						if (TrainTest == 0)		// if training?
						{
							for (unsigned long int i=0; i<(sizeof(histogramHOD)/sizeof(float)); i++)
							{
								createFileTrain<<" "<<i+1<<":"<<histogramHOD[i];
							}
							createFileTrain<<std::endl;
						}
						else if (TrainTest == 1)		// if testing?
						{
							for (unsigned long int i=0; i<(sizeof(histogramHOD)/sizeof(float)); i++)
								{
									createFileTest<<" "<<i+1<<":"<<histogramHOD[i];
								}
								createFileTest<<std::endl;
						}	
					}
				}
			}
		}
		if (TrainTest == 0)
			std::cout<<"hod_d2 has been created"<<std::endl;
		else if (TrainTest == 1)
			std::cout<<"hod_d2.t has been created"<<std::endl;
	}
	
	delete [] frameData;

	// scale files
	char scaleTrain[][1024] = {" ","-l","-1","-u","1","-s","range","hod_d2","hod_d2.scale"};
	char scaleTest[][1024] = {" ","-l","-1","-u","1","-s","range","hod_d2.t","hod_d2.t.scale"};
	svm_scale_function(9,scaleTrain);
	svm_scale_function(9,scaleTest);

	// train
	char training[][1024] = {" ","hod_d2.scale"};
	svm_train_function(2, training);

	// predict
	char predicting[][1024] = {" ","hod_d2.t.scale","hod_d2.scale.model","hod_d2.t.scale.predict"};
	svm_predict_function(4, predicting);

	return 0;
}

Frame_info* frameDataResize(Frame_info source[],unsigned int add_size)
{
	Frame_info* target = new Frame_info[malloc_usable_size(source)/sizeof(*source)+add_size];
	std::copy(source, source + malloc_usable_size(source)/sizeof(*source),target);
	delete [] source;
	return target;
}

float getMagnitude(float a, float b)
{
	return sqrt(pow(a,2)+pow(b,2));
}

float getAngle(float a, float b)
{
	return (atan2(b,a)+(M_PI))/(2*M_PI)*360;		// 0 ~ 2*pi => 0~360
}

void computeMagAng(MagAng_info magangData[], unsigned int frameNumber, Frame_info frameData[])
{
	for (unsigned int t = 1; t < frameNumber; t++)
	{
		magangData[t-1].Time_ID = t;
		for(unsigned int i=0; i<NO_JOINT; i++)
		{
			magangData[t-1].mag[i][0] = getMagnitude(frameData[t].xy[i][0]-frameData[t-1].xy[i][0],frameData[t].xy[i][1]-frameData[t-1].xy[i][1]);
			magangData[t-1].mag[i][1] = getMagnitude(frameData[t].yz[i][0]-frameData[t-1].yz[i][0],frameData[t].yz[i][1]-frameData[t-1].yz[i][1]);
			magangData[t-1].mag[i][2] = getMagnitude(frameData[t].xz[i][0]-frameData[t-1].xz[i][0],frameData[t].xz[i][1]-frameData[t-1].xz[i][1]);
			magangData[t-1].ang[i][0] = getAngle(frameData[t].xy[i][0]-frameData[t-1].xy[i][0],frameData[t].xy[i][1]-frameData[t-1].xy[i][1]);
			magangData[t-1].ang[i][1] = getAngle(frameData[t].yz[i][0]-frameData[t-1].yz[i][0],frameData[t].yz[i][1]-frameData[t-1].yz[i][1]);
			magangData[t-1].ang[i][2] = getAngle(frameData[t].xz[i][0]-frameData[t-1].xz[i][0],frameData[t].xz[i][1]-frameData[t-1].xz[i][1]);
		}
	}
}
						
void createHistogram(float histogramHOD[NO_JOINT][3][BIN_ANGLE], unsigned int start, unsigned int end, MagAng_info magangData[])
{
	for(unsigned int f = start; f <= end; f++)	// each frame
	{
		for (unsigned int j = 0; j < NO_JOINT; j++)	// each joints
		{
			for (unsigned int i = 0; i < 3; i++)	// xy, yz, xz
			{
				unsigned int bin = ((magangData[f].ang[j][i])/(360/BIN_ANGLE));
				histogramHOD[j][i][bin] = histogramHOD[j][i][bin] + magangData[f].mag[j][i];
			}
		}
	}
}

void normalizeHistogram(float histogramHOD[NO_JOINT][3][BIN_ANGLE])
{
	float total_magnitude = 0;
	
	for(unsigned int j=0; j<NO_JOINT; j++)
	{
		for(unsigned int i=0; i<3; i++)
		{
			for(unsigned int b=0; b<BIN_ANGLE; b++)
			{
				total_magnitude = total_magnitude + histogramHOD[j][i][b];
			}
			
			if(total_magnitude != 0)
			{
				for(unsigned int b=0; b<BIN_ANGLE; b++)
				{
					histogramHOD[j][i][b] = histogramHOD[j][i][b]/total_magnitude;
				}
			}
		}
	}
}

void concatenateHistogram(float histogramHOD[7*(NO_JOINT)*3*(BIN_ANGLE)], float histogramHOD_11[NO_JOINT][3][BIN_ANGLE], float histogramHOD_21[NO_JOINT][3][BIN_ANGLE], float histogramHOD_22[NO_JOINT][3][BIN_ANGLE], float histogramHOD_31[NO_JOINT][3][BIN_ANGLE], float histogramHOD_32[NO_JOINT][3][BIN_ANGLE], float histogramHOD_33[NO_JOINT][3][BIN_ANGLE], float histogramHOD_34[NO_JOINT][3][BIN_ANGLE])
{
	unsigned long int index = 0;
	unsigned int plus = (NO_JOINT)*3*(BIN_ANGLE);
	
	for(unsigned int j=0; j<NO_JOINT; j++)
	{
		for(unsigned int i=0; i<3; i++)
		{
			for(unsigned int b=0; b<BIN_ANGLE; b++)
			{
				histogramHOD[index] = histogramHOD_11[j][i][b];
				histogramHOD[index+plus] = histogramHOD_21[j][i][b];
				histogramHOD[index+2*plus] = histogramHOD_22[j][i][b];
				histogramHOD[index+3*plus] = histogramHOD_31[j][i][b];
				histogramHOD[index+4*plus] = histogramHOD_32[j][i][b];
				histogramHOD[index+5*plus] = histogramHOD_33[j][i][b];
				histogramHOD[index+6*plus] = histogramHOD_34[j][i][b];
				index++;
			}
		}
	}
}
