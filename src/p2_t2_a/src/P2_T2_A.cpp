#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <string>
#include <algorithm>
#include <malloc.h>
#include "scale.h"
#include "train.h"
#include "predict.h"

#define NO_JOINT 20	// The number of joints in a skeleton
#define CENTER_OF_HIP 0	// joint 1
#define HEAD 3		// joint 4
#define RIGHT_HAND 7 	// joint 8
#define LEFT_HAND 11	// joint 12
#define RIGHT_FOOT 15	// joint 16
#define LEFT_FOOT 19	// joint 20
#define FRAME_BUFFER_SIZE 100
#define BIN_N 8		// The number of bins for each d
#define BIN_M 8		// The number of bins for each theta

//using namespace std

struct Joint_info{
	unsigned int JointNumber;
	float x;
	float y;
	float z;
};

struct Frame_info{
	unsigned int Frame_ID;
	float d[5];		// d[0] = d1, d[1] = d2, d[2] = d3, d[3] = d4, d[4] = d5
	float theta[5];		// theta[0] = theta1, theta[1] = theta2, theta[2] = theta3, theta[3] = theta4, theta[4] = theta5
};

float getSize(std::vector<float> &vector1);
float getAngle(std::vector<float> &vector1, std::vector<float> &vector2);
Frame_info* frameDataResize(Frame_info source[],unsigned int add_size);
void vectorSet(std::vector<float> &vector1, std::vector<float> &vector2, std::vector<float> &vector3, std::vector<float> &vector4, std::vector<float> &vector5, const Joint_info[]);
void computeHistogram_d(Frame_info Data, unsigned int index,std::vector<float> &vector1, float startbin, float endbin, unsigned int No_bins);
void computeHistogram_theta(Frame_info Data, unsigned int index,std::vector<float> &vector1, float startbin, float endbin, unsigned int No_bins);
void normalizeHistogram(std::vector<float> &vector1, unsigned int No_bins);
void concatenateVectors(std::vector<float> &vectorOutput, std::vector<float> &vector1, std::vector<float> &vector2, std::vector<float> &vector3, std::vector<float> &vector4, std::vector<float> &vector5, std::vector<float> &vector6, std::vector<float> &vector7, std::vector<float> &vector8, std::vector<float> &vector9, std::vector<float> &vector10);
void createHistogram(std::ofstream& outputFile, std::vector<float> &vector1);

int main(int argc, char** argv) {

	// Initialize key variables
	unsigned int frame_size = FRAME_BUFFER_SIZE;
	Frame_info* frameData = new Frame_info[frame_size]();
	Joint_info joint[NO_JOINT];		// joint[0] = joint1, joint[1] = joint2, joint[2] = joint3, joint[3] = joint4, ...
	unsigned int frame;
	unsigned int jointnumber;
	
	// input file path
	std::string filePath[] = {"./dataset/train/a00_s00_e00_skeleton_proj.txt", "./dataset/test/a00_s00_e00_skeleton_proj.txt"};
	const std::string activity[] = {"01","02","03","04","05","06","07","08","09","10","11","12","13","14","15","16"};
	const std::string subject[] = {"01", "02", "03", "04", "05", "06", "07", "08", "09", "10"};
	const std::string trial[] = {"01","02"};

	// output file
	std::ofstream createFileTrain("rad_d2");
	std::ofstream createFileTest("rad_d2.t");

	for (int TrainTest=0; TrainTest<2; TrainTest++)		// TrainTest = 0 (Train), TrainTest = 1 (Test)
	{
		for(unsigned int k=0;k<16;k++)
		{
			// histogram vectors
			std::vector<float> vector_d1(BIN_N, 0.0);	std::vector<float> vector_theta1(BIN_M, 0.0);
			std::vector<float> vector_d2(BIN_N, 0.0);	std::vector<float> vector_theta2(BIN_M, 0.0);
			std::vector<float> vector_d3(BIN_N, 0.0);	std::vector<float> vector_theta3(BIN_M, 0.0);
			std::vector<float> vector_d4(BIN_N, 0.0);	std::vector<float> vector_theta4(BIN_M, 0.0);
			std::vector<float> vector_d5(BIN_N, 0.0);	std::vector<float> vector_theta5(BIN_M, 0.0);
		
			long unsigned int ID_number = 0;
		
			for(unsigned int j=0;j<10;j++)
			{
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

						std::vector<float> vector1(3);
						std::vector<float> vector2(3);
						std::vector<float> vector3(3);
						std::vector<float> vector4(3);
						std::vector<float> vector5(3);
					
						while(!openFile.eof())
						{
							if(ID_number == frame_size)
							{
								// add more buffer (FRAME_BUFFER_SIZE)
								frameData = frameDataResize(frameData, FRAME_BUFFER_SIZE);				
								frame_size = frame_size + FRAME_BUFFER_SIZE;
							}
						
							for(int i=0;i<NO_JOINT;i++)
							{
								openFile >> frame;
								openFile >> joint[i].JointNumber;
								openFile >> joint[i].x;
								openFile >> joint[i].y;
								openFile >> joint[i].z;
							}
						
							frameData[ID_number].Frame_ID = ID_number;
							vectorSet(vector1,vector2,vector3,vector4,vector5,joint);
			
							// distance calculation
							frameData[ID_number].d[0] = getSize(vector1);
							frameData[ID_number].d[1] = getSize(vector2);
							frameData[ID_number].d[2] = getSize(vector3);
							frameData[ID_number].d[3] = getSize(vector4);
							frameData[ID_number].d[4] = getSize(vector5);
						
							// angle calculation
							frameData[ID_number].theta[0] = getAngle(vector1,vector2);
							frameData[ID_number].theta[1] = getAngle(vector1,vector3);
							frameData[ID_number].theta[2] = getAngle(vector2,vector4);
							frameData[ID_number].theta[3] = getAngle(vector3,vector5);
							frameData[ID_number].theta[4] = getAngle(vector4,vector5);
						
							// Compute Histogram of each d
							computeHistogram_d(frameData[ID_number], 0, vector_d1, 0.3, 0.8, BIN_N);
							computeHistogram_d(frameData[ID_number], 1, vector_d2, 0.3, 0.8, BIN_N);
							computeHistogram_d(frameData[ID_number], 2, vector_d3, 0.3, 0.8, BIN_N);
							computeHistogram_d(frameData[ID_number], 3, vector_d4, 0.3, 0.8, BIN_N);
							computeHistogram_d(frameData[ID_number], 4, vector_d5, 0.3, 0.8, BIN_N);
						
							// Compute Histogram of each theta
							computeHistogram_theta(frameData[ID_number], 0, vector_theta1, 0, 1.57080, BIN_M);
							computeHistogram_theta(frameData[ID_number], 1, vector_theta2, 0, 1.57080, BIN_M);
							computeHistogram_theta(frameData[ID_number], 2, vector_theta3, 0, 1.57080, BIN_M);
							computeHistogram_theta(frameData[ID_number], 3, vector_theta4, 0, 1.57080, BIN_M);
							computeHistogram_theta(frameData[ID_number], 4, vector_theta5, 0, 1.57080, BIN_M);
						
							ID_number++;
						}
						openFile.close();

						// normalization of each d and theta
						normalizeHistogram(vector_d1, BIN_N); normalizeHistogram(vector_theta1, BIN_M);
						normalizeHistogram(vector_d2, BIN_N); normalizeHistogram(vector_theta2, BIN_M);
						normalizeHistogram(vector_d3, BIN_N); normalizeHistogram(vector_theta3, BIN_M);
						normalizeHistogram(vector_d4, BIN_N); normalizeHistogram(vector_theta4, BIN_M);
						normalizeHistogram(vector_d5, BIN_N); normalizeHistogram(vector_theta5, BIN_M);
		
						// concatenate all normalized histograms
						std::vector<float> vectorOutput(5*(BIN_N+BIN_M),0.0);
						concatenateVectors(vectorOutput, vector_d1, vector_d2, vector_d3, vector_d4, vector_d5, vector_theta1, vector_theta2, vector_theta3, vector_theta4, vector_theta5);	
		
						// create a single line vector
						if (TrainTest == 0)		// if training?
							createHistogram(createFileTrain, vectorOutput);
						else if (TrainTest == 1)		// if testing?
							createHistogram(createFileTest, vectorOutput);
					}
				}
			}
		}
		if (TrainTest == 0)
			std::cout<<"rad_d2 has been created"<<std::endl;
		else if (TrainTest == 1)
			std::cout<<"rad_d2.t has been created"<<std::endl;
	}

	delete [] frameData;

	// scale files
	char scaleTrain[][1024] = {" ","-l","-1","-u","1","-s","range","rad_d2","rad_d2.scale"};
	char scaleTest[][1024] = {" ","-l","-1","-u","1","-s","range","rad_d2.t","rad_d2.t.scale"};
	svm_scale_function(9,scaleTrain);
	svm_scale_function(9,scaleTest);

	// train
	char training[][1024] = {" ","rad_d2.scale"};
	svm_train_function(2, training);

	// predict
	char predicting[][1024] = {" ","rad_d2.t.scale","rad_d2.scale.model","rad_d2.t.scale.predict"};
	svm_predict_function(4, predicting);

	return 0;
}

float getSize(std::vector<float> &vectorData)
{
	return sqrt(pow(vectorData.at(0),2) + pow(vectorData.at(1),2) + pow(vectorData.at(2),2));
}

float getAngle(std::vector<float> &vector1, std::vector<float> &vector2)
{
	return acos((vector1.at(0)*vector2.at(0)+vector1.at(1)*vector2.at(1)+vector1.at(2)*vector2.at(2))/(getSize(vector1)*getSize(vector2)));
}

Frame_info* frameDataResize(Frame_info source[],unsigned int add_size)
{
	Frame_info* target = new Frame_info[malloc_usable_size(source)/sizeof(*source)+add_size];
	std::copy(source, source + malloc_usable_size(source)/sizeof(*source),target);
	delete [] source;
	return target;
}

void vectorSet(std::vector<float> &vector1,std::vector<float> &vector2,std::vector<float> &vector3,std::vector<float> &vector4,std::vector<float> &vector5,const Joint_info joint[])
{
	vector1.at(0)=joint[HEAD].x-joint[CENTER_OF_HIP].x;
	vector1.at(1)=joint[HEAD].y-joint[CENTER_OF_HIP].y;
	vector1.at(2)=joint[HEAD].z-joint[CENTER_OF_HIP].z;
	
	vector2.at(0)=joint[RIGHT_HAND].x-joint[CENTER_OF_HIP].x;
	vector2.at(1)=joint[RIGHT_HAND].y-joint[CENTER_OF_HIP].y;
	vector2.at(2)=joint[RIGHT_HAND].z-joint[CENTER_OF_HIP].z;
	
	vector3.at(0)=joint[LEFT_HAND].x-joint[CENTER_OF_HIP].x;
	vector3.at(1)=joint[LEFT_HAND].y-joint[CENTER_OF_HIP].y;
	vector3.at(2)=joint[LEFT_HAND].z-joint[CENTER_OF_HIP].z;
	
	vector4.at(0)=joint[RIGHT_FOOT].x-joint[CENTER_OF_HIP].x;
	vector4.at(1)=joint[RIGHT_FOOT].y-joint[CENTER_OF_HIP].y;
	vector4.at(2)=joint[RIGHT_FOOT].z-joint[CENTER_OF_HIP].z;
	
	vector5.at(0)=joint[LEFT_FOOT].x-joint[CENTER_OF_HIP].x;
	vector5.at(1)=joint[LEFT_FOOT].y-joint[CENTER_OF_HIP].y;
	vector5.at(2)=joint[LEFT_FOOT].z-joint[CENTER_OF_HIP].z;
}

void computeHistogram_d(Frame_info Data, unsigned int index,std::vector<float> &vector1,  float startbin, float endbin, unsigned int No_bins)
{
	float delta_bin = (endbin-startbin)/No_bins;
	
	for(int i=0; i<No_bins; i++)
	{
		if(Data.d[index] <= (startbin+(delta_bin*(i+1))))
		{
			vector1.at(i)++;
			break;
		}
		if(i == No_bins-1)
		{

			vector1.at(i)++;
		}
	}
}

void computeHistogram_theta(Frame_info Data, unsigned int index,std::vector<float> &vector1,  float startbin, float endbin, unsigned int No_bins)
{
	float delta_bin = (endbin-startbin)/No_bins;
	
	for(int i=0; i<No_bins; i++)
	{
		if(Data.theta[index] <= (startbin+(delta_bin*(i+1))))
		{
			vector1.at(i)++;
			break;
		}
		if(i == No_bins-1)
		{

			vector1.at(i)++;
		}
	}
}

void normalizeHistogram(std::vector<float> &vector1, unsigned int No_bins)
{
	unsigned int total_frame = 0;
	
	for(unsigned int i=0;i<No_bins;i++)
	{
		total_frame = total_frame+vector1.at(i);
	}
	
	if(total_frame != 0)
	{
		for(unsigned int i=0;i<No_bins;i++)
		{
			vector1.at(i) = vector1.at(i)/total_frame;
		}
	}
}

void concatenateVectors(std::vector<float> &vectorOutput, std::vector<float> &vector1, std::vector<float> &vector2, std::vector<float> &vector3, std::vector<float> &vector4, std::vector<float> &vector5, std::vector<float> &vector6, std::vector<float> &vector7, std::vector<float> &vector8, std::vector<float> &vector9, std::vector<float> &vector10)
{	
	for(unsigned int i=0; i<BIN_N; i++)
	{
		vectorOutput.at(i) = vector1.at(i);
		vectorOutput.at(i+(BIN_N+BIN_M)) = vector2.at(i);
		vectorOutput.at(i+(BIN_N+BIN_M)*2) = vector3.at(i);
		vectorOutput.at(i+(BIN_N+BIN_M)*3) = vector4.at(i);
		vectorOutput.at(i+(BIN_N+BIN_M)*4) = vector5.at(i);
	}
	
	for(unsigned int i=0; i<BIN_M; i++)
	{
		vectorOutput.at(i+BIN_N) = vector6.at(i);
		vectorOutput.at(i+(BIN_N+BIN_M)) = vector7.at(i);
		vectorOutput.at(i+(BIN_N+BIN_M)*2) = vector8.at(i);
		vectorOutput.at(i+(BIN_N+BIN_M)*3) = vector9.at(i);
		vectorOutput.at(i+(BIN_N+BIN_M)*4) = vector10.at(i);
	}
}

void createHistogram(std::ofstream& outputFile, std::vector<float> &vector1)
{
	for(unsigned int i=0;i<5*(BIN_N+BIN_M);i++)
	{
		outputFile<<" "<<i+1<<":"<<vector1.at(i);
	}
	outputFile<<std::endl;
}

