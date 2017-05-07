#include <tistdtypes.h>
#include <coecsl.h>
#include "user_includes.h"
#include "math.h"

// These two offsets are only used in the main file user_CRSRobot.c  You just need to create them here and find the correct offset and then these offset will adjust the encoder readings
float offset_Enc2_rad = -0.4238;
float offset_Enc3_rad = 0.2571;


// Your global varialbes.  

long mycount = 0;

#pragma DATA_SECTION(whattoprint, ".my_vars") ///visible by matlab
float whattoprint = 0.0;

#pragma DATA_SECTION(theta1array, ".my_arrs")	//visible by matlab
float theta1array[100];

#pragma DATA_SECTION(theta2array, ".my_arrs")	//visible by matlab

float theta2array[100];
long arrayindex = 0;

// Printing Values
float printtheta1motor = 0;
float printtheta2motor = 0;
float printtheta3motor = 0;
float printtheta1dh = 0;
float printtheta2dh = 0;
float printtheta3dh = 0;
float printendx = 0;
float printendy = 0;
float printendz = 0;
float printtheta1inv = 0;
float printtheta2inv = 0;
float printtheta3inv = 0;

// Assign these float to the values you would like to plot in Simulink
float Simulink_PlotVar1 = 0;
float Simulink_PlotVar2 = 0;
float Simulink_PlotVar3 = 0;
float Simulink_PlotVar4 = 0;


////////////////////// Lab 3 Coefficients for Friction Compensation
float Viscous_positive[3] = {0.17, 0.23, 0.17};
float Viscous_negative[3] = {0.2, 0.25, 0.2};
float Coulombs_positive[3] = {0.36, 0.4759, 0.5339};
float Coulombs_negative[3] = {-0.2948, -0.5031, -0.5190};
float slope_between_minimums[3] = {3.6, 3.6, 3.6};

////////////////// Vector to Store the Friction Values
float u_fric[3] = {0,0,0};

////////////////// Filtering for Calculating the Theta Derivatives
float Theta1_old = 0;
float Omega1_raw = 0;
float Omega1_old1 = 0;
float Omega1_old2 = 0;
float Omega1 = 0;

float Theta2_old = 0;
float Omega2_raw = 0;
float Omega2_old1 = 0;
float Omega2_old2 = 0;
float Omega2 = 0;

float Theta3_old = 0;
float Omega3_raw = 0;
float Omega3_old1 = 0;
float Omega3_old2 = 0;
float Omega3 = 0;

////////////////// Lab4  Variables Starts Here!

// Variables for Filtering x_dots
float x_dot_old1 = 0, x_dot_old2 = 0, y_dot_old1 = 0, y_dot_old2 = 0, z_dot_old1 = 0, z_dot_old2 = 0;
float x_dot_raw = 0, y_dot_raw = 0, z_dot_raw = 0;
float x_dot = 0, y_dot = 0, z_dot = 0; 	//position derivative
float x_old = 0, y_old = 0, z_old = 0;

// J Matrix
float J[3][3];
// F Matrix
float F[3];

// Desired End Effector World Frame Coordinates
float xd = 0, yd = 0, zd = 0;
float xd_dot = 0, yd_dot = 0, zd_dot = 0;
// Parameter Adjusting Friction Values
float fric_percent = 1;

// The World Kp and Kd Values
float Kpx = 0.8;
float Kpy = 0.8;
float Kpz = 0.8;
float Kdx = 0.05;
float Kdy = 0.05;
float Kdz = 0.05;

/////////////////// Lab 4 Part 2 Impedance Control
// Input angles values to Save Calls to cos() and sin()
float cosz = 0, sinz = 0, cosx = 0, sinx = 0, cosy = 0, siny = 0;

// Frame Rotation Angles
float thetax = 0;
float thetay = 0;
float thetaz = PI/4; // Our Trajactory Frame is rotation around the World Z-axis

// Variables for Rotation Matrix from Need to World
float R11 = 0, R12 = 0, R13 = 0, R21 = 0, R22 = 0, R23 = 0, R31 = 0, R32 = 0, R33 = 0;
float RT11 = 0, RT12 = 0, RT13 = 0, RT21 = 0, RT22 = 0, RT23 = 0, RT31 = 0, RT32 = 0, RT33 = 0;

// Variables for Calculating Force in World Frame
float Fxw = 0, Fyw = 0, Fzw = 0;

// Control Parameters Kp and Kd
float Kpxn = 0.8;
float Kpyn = 0.1;
float Kpzn = 0.8;
float Kdxn = 0.05;
float Kdyn = 0.01;
float Kdzn = 0.05;

// Time Variable for Trajectory Calculationg
float time = 0;


// This function calculates real time (xd yd zd) and (xd_dot yd_dot zd_dot) for a given
// straight line trajectory in part III. It updates variableseverytime called in the main function
// The input are the starting and ending coordinates of the straight line in world frame
void straightline(float xa, float ya, float za, float xb, float yb, float zb, float speed,
		float t)
{
	float distance = sqrt((xb-xa)*(xb-xa) +  (yb-ya)*(yb-ya) + (zb-za)*(zb-za));
	float t_total = distance/speed;
	float dx = xb-xa;
	float dy = yb-ya;
	float dz = zb-za;
	// Set End Effector to stop at ending point
	if (t > t_total)
	{
		xd = xb;
		yd = yb;
		zd = zb;
		xd_dot = yd_dot = zd_dot = 0;
		return;
	}
	//Calculating desired xd yd zd
	xd = dx*(t)/t_total + xa;
	yd = dy*(t)/t_total + ya;
	zd = dz*(t)/t_total + za;
	//calculating desired xd_dot, yd_dot, zd_dot
	xd_dot = (xb-xa)/t_total;
	yd_dot = (yb-ya)/t_total;
	zd_dot = (zb-za)/t_total;
	return;
}


// This function is called every 1 ms
void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int error) {

	*tau1 = 0;
	*tau2 = 0;
	*tau3 = 0;

	//Motor torque limitation(Max: 5 Min: -5)

	// Forward Kinematics
	float endx = 10*cos(theta1motor)*(cos(theta3motor) + sin(theta2motor));
	float endy = 10*sin(theta1motor)*(cos(theta3motor) + sin(theta2motor));
	float endz = 10*(1+cos(theta2motor)-sin(theta3motor));

	// save past states
	if ((mycount%50)==0) {

		theta1array[arrayindex] = theta1motor;
		theta2array[arrayindex] = theta2motor;

		if (arrayindex >= 100) {
			arrayindex = 0;
		} else {
			arrayindex++;
		}

	}

	/////////////// Section For Calculating Friction Compensation /////////////////
	// In this section, we calculate the friction compensation values directly from
	// the current theta velocities. The Parameters are tuned in Lab 3 to make the
	// Calculated Values match our robot arm. In this section most of the code that we
	// used are given in the lab manual

	// Filtering of Motor Theta 1
	Omega1 = (theta1motor - Theta1_old)/0.001;
	Omega1 = (Omega1 + Omega1_old1 + Omega1_old2)/3.0;
	Theta1_old = theta1motor;
	Omega1_old2 = Omega1_old1;
	Omega1_old1 = Omega1;

	// Filtering of Motor Theta 2
	Omega2 = (theta2motor - Theta2_old)/0.001;
	Omega2 = (Omega2 + Omega2_old1 + Omega2_old2)/3.0;
	Theta2_old = theta2motor;
	Omega2_old2 = Omega2_old1;
	Omega2_old1 = Omega2;

	// Filtering of Motor Theta 3
	Omega3 = (theta3motor - Theta3_old)/0.001;
	Omega3 = (Omega3 + Omega3_old1 + Omega3_old2)/3.0;
	Theta3_old = theta3motor;
	Omega3_old2 = Omega3_old1;
	Omega3_old1 = Omega3;

	// Follow the Lab Description To Compute and Store in u_fric
	// Saved as arrays to make loop easier and efficient
	float Omega[3] = {Omega1, Omega2, Omega3};
	float min_vel[3] = {0.1, 0.05, 0.05};
	int i;
	// Loop for the three Joints
	for(i = 0; i<3; ++i)
	{
		float joint_velocity = Omega[i];
		float minimum_velocity = min_vel[i];
		if (joint_velocity >= minimum_velocity) {
			u_fric[i] = Viscous_positive[i]*joint_velocity + Coulombs_positive[i] ;
			} else if (joint_velocity <= -minimum_velocity) {
			u_fric[i] = Viscous_negative[i]*joint_velocity + Coulombs_negative[i];
			} else {
			u_fric[i] = slope_between_minimums[i]*joint_velocity;
		}
	}

	// Adding Friction Compensation to the Torque
	*tau1 += fric_percent*u_fric[0];
	*tau2 += fric_percent*u_fric[1];
	*tau3 += fric_percent*u_fric[2];

	/////////////// Section For Calculating None-Friction Torque /////////////////
	// In this part, we calculated the None-Friction Torque needed to control the trajectory
	// of the end effector. We will calculate the Jacobian Matrix, Rotation Matrix in this
	// Section while minimizing the call to sin() and cos(). Also we call our function to
	// generate the straight line trajectory and use them to calculate the final torque.

	// PreCalculated Trig Values to Save Calls to sin() and cos()
	float sine1 = sin(theta1motor);
	float sine2 = sin(theta2motor);
	float sine3 = sin(theta3motor);
	float cosine1 = cos(theta1motor);
	float cosine2 = cos(theta2motor);
	float cosine3 = cos(theta3motor);

	// Calculating the Jacobian Values (Index already Transposed)
	J[0][0] = -10*sine1*(cosine3 + sine2);
	J[1][0] = 10*cosine1 * (cosine3 + sine2);
	J[2][0] = 0;
	J[0][1] = 10*cosine1*(cosine2 - sine3);
	J[1][1] = 10*sine1*(cosine2 - sine3);
	J[2][1] = -10*(cosine3 + sine2);
	J[0][2] = -10*cosine1*sine3;
	J[1][2] = -10*sine1*sine3;
	J[2][2] = -10*cosine3;

	//  Filtering the x_dot
	x_dot = (endx - x_old)/0.001;
	x_dot = (x_dot + x_dot_old1 + x_dot_old2)/3.0;
	x_dot_old2 = x_dot_old1;
	x_dot_old1 = x_dot;
	x_old = endx;

	//  Filtering the y_dot
	y_dot = (endy - y_old)/0.001;
	y_dot = (y_dot + y_dot_old1 + y_dot_old2)/3.0;
	y_dot_old2 = y_dot_old1;
	y_dot_old1 = y_dot;
	y_old = endy;

	//  Filtering the z_dot
	z_dot = (endz - z_old)/0.001;
	z_dot = (z_dot + z_dot_old1 + z_dot_old2)/3.0;
	z_dot_old2 = z_dot_old1;
	z_dot_old1 = z_dot;
	z_old = endz;

	// Copied from Lab 4 Manual
	// PreCalculated Trig Values to Save Calls to sin() and cos()
	cosz = cos(thetaz);
	sinz = sin(thetaz);
	cosx = cos(thetax);
	sinx = sin(thetax);
	cosy = cos(thetay);
	siny = sin(thetay);

	// Rotation Matrix Entry Calculation: R stands for Rwn
	RT11 = R11 = cosz*cosy-sinz*sinx*siny;
	RT21 = R12 = -sinz*cosx;
	RT31 = R13 = cosz*siny+sinz*sinx*cosy;
	RT12 = R21 = sinz*cosy+cosz*sinx*siny;
	RT22 = R22 = cosz*cosx;
	RT32 = R23 = sinz*siny-cosz*sinx*cosy;
	RT13 = R31 = -cosx*siny;
	RT23 = R32 = sinx;
	RT33 = R33 = cosx*cosy;

	// Part III: calling straightline function to update xd yd zd and corresponding derivaties
	// Call straightline() to return the straight line trajectory
	// time: The Current System Time in seconds
	time = (mycount%10000)/1000.0;
	// The First Five Seconds Going from (8,-2, 8) -> (15, 5, 8) with speed 4 inches/seconds
	if (time < 5){
		time = time>2? time-2:0;
		straightline(8, -2, 8, 15, 5, 8, 4, time);
	}
	// The Second Five Seconds Going from (15, 5, 8) -> (8,-2, 8) with speed 4 inches/seconds
	else
	{
		time = time>=6? time-6:0;
		straightline(15, 5, 8, 8, -2, 8, 4, time);
	}

	// vector F for part 1 without coordinate transformation
	/*
	F[0] = Kpxn*(xd - endx) + Kdxn*(xd_dot - x_dot);
	F[1] = Kpyn*(yd - endy) + Kdyn*(yd_dot - y_dot);
	F[2] = Kpzn*(zd - endz) + Kdzn*(zd_dot - z_dot);
	*/

	// The following part is pure matrix multiplication
	// Calculate F for part III: Coordinate Rotation Transformation from W to N frame
	F[0] = Kpxn*(RT11*(xd - endx) + RT12*(yd - endy) + RT13*(zd - endz) )
		   + Kdxn*(RT11*(xd_dot - x_dot) + RT12*(yd_dot - y_dot) + RT13*(zd_dot - z_dot));
	F[1] = Kpyn*(RT21*(xd - endx) + RT22*(yd - endy) + RT23*(zd - endz) )
		   + Kdyn*(RT21*(xd_dot - x_dot) + RT22*(yd_dot - y_dot) + RT23*(zd_dot - z_dot));
	F[2] = Kpzn*(RT31*(xd - endx) + RT32*(yd - endy) + RT33*(zd - endz))
		   + Kdzn*(RT31*(xd_dot - x_dot) + RT32*(yd_dot - y_dot) + RT33*(zd_dot - z_dot));

	// Calculate F for Part III: Coordinate Rotation Transformation from W to N frame
	Fxw = R11*F[0] + R12*F[1] + R13*F[2];
	Fyw = R21*F[0] + R22*F[1] + R23*F[2];
	Fzw = R31*F[0] + R32*F[1] + R33*F[2];

	// Torque values without friction compensations
	*tau1 += J[0][0] * Fxw + J[1][0] * Fyw + J[2][0] * Fzw;
	*tau2 += J[0][1] * Fxw + J[1][1] * Fyw + J[2][1] * Fzw;
	*tau3 += J[0][2] * Fxw + J[1][2] * Fyw + J[2][2] * Fzw;

	// From Lab 1
	if ((mycount%500)==0) {
		if (error != 0){
			serial_printf(&SerialA, "error position\n\r");
		}
		else{
		if (whattoprint > 0.5) {
			serial_printf(&SerialA, "I love robotics\n\r");
		} else {
			printtheta1motor = theta1motor;
			printtheta2motor = theta2motor;
			printtheta3motor = theta3motor;
			SWI_post(&SWI_printf); //Using a SWI to fix SPI issue from sending too many floats.
		}}
		GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Blink LED on Control Card
		GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1; // Blink LED on Emergency Stop Box
	}
	mycount++;
}

void printing(void){
	serial_printf(&SerialA, "Motor Angle: %.2f %.2f,%.2f   \n\r",printtheta1motor*180/PI,printtheta2motor*180/PI,printtheta3motor*180/PI);
	//serial_printf(&SerialA, "Joint Angle: %.2f %.2f,%.2f   \n\r",printtheta1dh*180/PI,printtheta2dh*180/PI,printtheta3dh*180/PI);
	//serial_printf(&SerialA, "End Position: %.2f %.2f,%.2f   \n\r",printendx,printendy,printendz);
	//serial_printf(&SerialA, "Inverse Angle: %.2f %.2f,%.2f   \n\r",printtheta1inv*180/PI,printtheta2inv*180/PI,printtheta3inv*180/PI);
}

