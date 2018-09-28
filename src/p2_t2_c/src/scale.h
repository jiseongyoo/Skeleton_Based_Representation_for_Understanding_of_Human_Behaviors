#ifndef SCALE_H
#define SCALE_H

namespace scale
{
	extern char *line;
	extern int max_line_len;
	char* readline(FILE *input);
}

extern double lower,upper,y_lower,y_upper;
extern int y_scaling;
extern double *feature_max;
extern double *feature_min;
extern double y_max;
extern double y_min;
extern int max_index;
extern int min_index;
extern long int num_nonzeros;
extern long int new_num_nonzeros;


#define max(x,y) (((x)>(y))?(x):(y))
#define min(x,y) (((x)<(y))?(x):(y))

void scale_exit_with_help();
void output_target(double value);
void output(int index, double value, FILE output);
int clean_up(FILE *fp_restore, FILE *fp, const char *msg);
int svm_scale_function(int argc,char argv[][1024]);
void output_target(double value);
void output(int index, double value);
int clean_up(FILE *fp_restore, FILE *fp, const char* msg);

#endif
