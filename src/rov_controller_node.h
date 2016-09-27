enum trusters {top_left, top_right, middle_middle, bottom, middle_right, middle_left};

int fd;
bool ports_initialized = false;
bool close_loop = false;
bool reset_odometry = true;

float x_ref;
float y_ref;
float z_ref;

float x_angle_ref;
float y_angle_ref;
float z_angle_ref;


float x_ref_dot;
float y_ref_dot;
float z_ref_dot;

float x_angle_ref_dot;
float y_angle_ref_dot;
float z_angle_ref_dot;




//##   PID TUNING PARAMETERS    ##
	float kp_linear_x = 1;
	float kd_linear_x = 1;
	float ki_linear_x = 1;

	float kp_linear_y = 1;
	float kd_linear_y = 1;
	float ki_linear_y = 1;

	float kp_linear_z = 1;
	float kd_linear_z = 1;
	float ki_linear_z = 1;


	float kp_angular_x = 1;
	float kd_angular_x = 1;
	float ki_angular_x = 1;

	float kp_angular_y = 1;
	float kd_angular_y = 1;
	float ki_angular_y = 1;

	float kp_angular_z = 1;
	float kd_angular_z = 1;
	float ki_angular_z = 1;
//##                             ##
