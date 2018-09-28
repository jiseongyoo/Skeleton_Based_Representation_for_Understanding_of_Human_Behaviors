#include <iostream>
#include <fstream>
#include <math.h>
#include <string>
#include <algorithm>
#include <malloc.h>
#include "scale.h"
#include "train.h"
#include "predict.h"

#define NO_JOINT 20		// The number of joints in a skeleton
#define REF_JOINT 1		// Reference joint for HJPD
#define FRAME_BUFFER_SIZE 100
#define BIN_X 10	// The number of bins for each x displacement
#define BIN_Y 10	// The number of bins for each y displacement
#define BIN_Z 10	// The number of bins for each z displacement

//using namespace std

struct Joint_info{
	unsigned int JointNumber;
	float x;
	float y;
	float z;
};

struct Frame_info{
	unsigned int Frame_ID;
	float disp_x[NO_JOINT];
	float disp_y[NO_JOINT];
	float disp_z[NO_JOINT];
};

Frame_info* frameDataResize(Frame_info source[],unsigned int add_size);
void computeHistogram_x(Frame_info Data, float histogram_x[][BIN_X], unsigned int joint_num, float startbin, float endbin);
void computeHistogram_y(Frame_info Data, float histogram_y[][BIN_Y], unsigned int joint_num, float startbin, float endbin);
void computeHistogram_z(Frame_info Data, float histogram_z[][BIN_Z], unsigned int joint_num, float startbin, float endbin);
void normalizeHistogram_x(float histogram[][BIN_X]);
void normalizeHistogram_y(float histogram[][BIN_Y]);
void normalizeHistogram_z(float histogram[][BIN_Z]);
void concatenateHistogram(float histogramOutput[], float histogram_x[][BIN_X], float histogram_y[][BIN_Y], float histogram_z[][BIN_Z]);

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
	std::ofstream createFileTrain("hjpd_d2");
	std::ofstream createFileTest("hjpd_d2.t");
	
	for (int TrainTest=0; TrainTest<2; TrainTest++)		// TrainTest = 0 (Train), TrainTest = 1 (Test)
	{
		// activity
		for(unsigned int k=0;k<16;k++)
		{
			// histogram array
			float histogram_x[NO_JOINT][BIN_X] = {0.0};
			float histogram_y[NO_JOINT][BIN_Y] = {0.0};
			float histogram_z[NO_JOINT][BIN_Z] = {0.0};
		
			long unsigned int ID_number = 0;
		
			// subject
			for(unsigned int j=0;j<10;j++)
			{
				// trial
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
							if(ID_number == frame_size)
							{
								// add more buffer (FRAME_BUFFER_SIZE)
								frameData = frameDataResize(frameData, FRAME_BUFFER_SIZE);				
								frame_size = frame_size + FRAME_BUFFER_SIZE;
							}
						
							for(unsigned int i=0;i<NO_JOINT;i++)
							{
								openFile >> frame;
								openFile >> joint[i].JointNumber;
								openFile >> joint[i].x;
								openFile >> joint[i].y;
								openFile >> joint[i].z;
							}
						
							frameData[ID_number].Frame_ID = ID_number;
						
							// compute displacement x, y, z
							for(unsigned int i=0;i<NO_JOINT;i++)
							{
								frameData[ID_number].disp_x[i] = joint[i].x - joint[REF_JOINT-1].x;
								frameData[ID_number].disp_y[i] = joint[i].y - joint[REF_JOINT-1].y;
								frameData[ID_number].disp_z[i] = joint[i].z - joint[REF_JOINT-1].z;
							}
	
							// compute histogram of each joint x, y ,z
							for (unsigned int joint_num = 1;joint_num <= NO_JOINT; joint_num++)
							{
								computeHistogram_x(frameData[ID_number], histogram_x, joint_num, -0.5, 0.5);
								computeHistogram_y(frameData[ID_number], histogram_y, joint_num, -0.5, 0.5);
								computeHistogram_z(frameData[ID_number], histogram_z, joint_num, -0.5, 0.5);
							}

							ID_number++;
						}
						openFile.close();

						// normalize histogram of each x, y, z
						normalizeHistogram_x(histogram_x);
						normalizeHistogram_y(histogram_y);
						normalizeHistogram_z(histogram_z);
		
						// concatenate histogram of each x, y, z
						unsigned int outputSize = (BIN_X+BIN_Y+BIN_Z)*NO_JOINT;
						float histogramOutput[outputSize];
						concatenateHistogram(histogramOutput, histogram_x, histogram_y, histogram_z);
		
						// generate training file
						if(TrainTest == 0)		// if training?
						{
							for (unsigned int i = 0; i<outputSize; i++)
							{
								createFileTrain<<" "<<i+1<<":"<<histogramOutput[i];
							}
							createFileTrain<<std::endl;
						}
						else if(TrainTest == 1)			// if testing?
						{
							for (unsigned int i = 0; i<outputSize; i++)
							{
								createFileTest<<" "<<i+1<<":"<<histogramOutput[i];
							}
							createFileTest<<std::endl;
						}
					}
				}
			}
		}
		if (TrainTest == 0)
			std::cout<<"hjpd_d2 has been created"<<std::endl;
		else if (TrainTest == 1)
			std::cout<<"hjpd_d2.t has been created"<<std::endl;
	}
	delete [] frameData;
	
	// scale files
	char scaleTrain[][1024] = {" ","-l","-1","-u","1","-s","range","hjpd_d2","hjpd_d2.scale"};
	char scaleTest[][1024] = {" ","-l","-1","-u","1","-s","range","hjpd_d2.t","hjpd_d2.t.scale"};
	svm_scale_function(9,scaleTrain);
	svm_scale_function(9,scaleTest);

	// train
	char training[][1024] = {" ","hjpd_d2.scale"};
	svm_train_function(2, training);

	// predict
	char predicting[][1024] = {" ","hjpd_d2.t.scale","hjpd_d2.scale.model","hjpd_d2.t.scale.predict"};
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

void computeHistogram_x(Frame_info Data, float histogram_x[NO_JOINT][BIN_X], unsigned int joint_num, float startbin, float endbin)
{
	float delta_bin = (endbin-startbin)/BIN_X;
	
	for(unsigned int i=0; i<BIN_X; i++)
	{
		if(Data.disp_x[joint_num-1] <= (startbin+(delta_bin*(i+1))))
		{
			histogram_x[joint_num-1][i]++;
			break;
		}
		if(i == (BIN_X-1))
		{
			histogram_x[joint_num-1][i]++;
		}
	}
}

void computeHistogram_y(Frame_info Data, float histogram_y[NO_JOINT][BIN_Y], unsigned int joint_num, float startbin, float endbin)
{
	float delta_bin = (endbin-startbin)/BIN_Y;
	
	for(unsigned int i=0; i<BIN_Y; i++)
	{
		if(Data.disp_y[joint_num-1] <= (startbin+(delta_bin*(i+1))))
		{
			histogram_y[joint_num-1][i]++;
			break;
		}
		if(i == (BIN_Y-1))
		{
			histogram_y[joint_num-1][i]++;
		}
	}
}

void computeHistogram_z(Frame_info Data, float histogram_z[NO_JOINT][BIN_Z], unsigned int joint_num, float startbin, float endbin)
{
	float delta_bin = (endbin-startbin)/BIN_Z;
	
	for(unsigned int i=0; i<BIN_Z; i++)
	{
		if(Data.disp_z[joint_num-1] <= (startbin+(delta_bin*(i+1))))
		{
			histogram_z[joint_num-1][i]++;
			break;
		}
		if(i == (BIN_Z-1))
		{
			histogram_z[joint_num-1][i]++;
		}
	}
}

void normalizeHistogram_x(float histogram[][BIN_X])
{
	for (unsigned int j=0; j<NO_JOINT;j++)
	{
		unsigned int total_frame = 0;

		for(unsigned int i=0;i<BIN_X;i++)
		{
			total_frame = total_frame+histogram[j][i];
		}
		
		if(total_frame != 0)
		{
			for(unsigned int i=0;i<BIN_X;i++)
			{
				histogram[j][i] = histogram[j][i]/total_frame;
			}
		}
	}
}

void normalizeHistogram_y(float histogram[][BIN_Y])
{
	unsigned int total_frame = 0;
	
	for (unsigned int j=0; j<NO_JOINT;j++)
	{
		for(unsigned int i=0;i<BIN_Y;i++)
		{
			total_frame = total_frame+histogram[j][i];
		}
		
		if(total_frame != 0)
		{
			for(unsigned int j=0;j<NO_JOINT;j++)
			{
				for(unsigned int i=0;i<BIN_Y;i++)
				{
					histogram[j][i] = histogram[j][i]/total_frame;
				}
			}
		}
	}
}

void normalizeHistogram_z(float histogram[][BIN_Z])
{
	unsigned int total_frame = 0;
	
	for (unsigned int j=0; j<NO_JOINT;j++)
	{
		for(unsigned int i=0;i<BIN_Z;i++)
		{
			total_frame = total_frame+histogram[j][i];
		}
		
		if(total_frame != 0)
		{
			for(unsigned int j=0;j<NO_JOINT;j++)
			{
				for(unsigned int i=0;i<BIN_Z;i++)
				{
					histogram[j][i] = histogram[j][i]/total_frame;
				}
			}
		}
	}
}

void concatenateHistogram(float histogramOutput[], float histogram_x[][BIN_X], float histogram_y[][BIN_Y], float histogram_z[][BIN_Z])
{
	unsigned int histogramOutput_index = 0;
	
	for (unsigned int j=0;j<(NO_JOINT);j++)
	{
		for (unsigned int i=0;i<BIN_X;i++)
		{
			histogramOutput[histogramOutput_index] = histogram_x[j][i];
			histogramOutput_index++;
		}
		for (unsigned int i=0;i<BIN_Y;i++)
		{
			histogramOutput[histogramOutput_index] = histogram_y[j][i];
			histogramOutput_index++;
		}
		for (unsigned int i=0;i<BIN_Z;i++)
		{
			histogramOutput[histogramOutput_index] = histogram_z[j][i];
			histogramOutput_index++;
		}
	}
}
