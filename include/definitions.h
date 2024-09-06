#include "pros/motors.hpp"
#include "pros/adi.hpp"
#include "pros/rotation.hpp"
#include <cmath>

//Motor ports
#define LEFT_UPPER_BEVEL_MOTOR_1 2
#define LEFT_UPPER_BEVEL_MOTOR_2 13
#define LEFT_LOWER_BEVEL_MOTOR_1 1
#define LEFT_LOWER_BEVEL_MOTOR_2 12
#define RIGHT_UPPER_BEVEL_MOTOR_1 9
#define RIGHT_UPPER_BEVEL_MOTOR_2 19
#define RIGHT_LOWER_BEVEL_MOTOR_1 10
#define RIGHT_LOWER_BEVEL_MOTOR_2 20

//Rotational sensor ports
#define LEFT_ROTATION_SENSOR_PORT 14
#define RIGHT_ROTATION_SENSOR_PORT 18

#define ZERO_VECTOR INFINITY

//Controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

//Individual motors
pros::Motor luA(LEFT_UPPER_BEVEL_MOTOR_1, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor luB(LEFT_UPPER_BEVEL_MOTOR_2, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor llA(LEFT_LOWER_BEVEL_MOTOR_1, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor llB(LEFT_LOWER_BEVEL_MOTOR_2, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor ruA(RIGHT_UPPER_BEVEL_MOTOR_1, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor ruB(RIGHT_UPPER_BEVEL_MOTOR_2, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rlA(RIGHT_LOWER_BEVEL_MOTOR_1, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rlB(RIGHT_LOWER_BEVEL_MOTOR_2, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);

//Grouped motors (upper and lower)
pros::MotorGroup lu({LEFT_UPPER_BEVEL_MOTOR_1, LEFT_UPPER_BEVEL_MOTOR_2}, pros::E_MOTOR_GEARSET_06, pros::E_MOTOR_ENCODER_DEGREES);
pros::MotorGroup ll({LEFT_LOWER_BEVEL_MOTOR_1, LEFT_LOWER_BEVEL_MOTOR_2}, pros::E_MOTOR_GEARSET_06, pros::E_MOTOR_ENCODER_DEGREES);
pros::MotorGroup ru({RIGHT_UPPER_BEVEL_MOTOR_1, RIGHT_UPPER_BEVEL_MOTOR_2}, pros::E_MOTOR_GEARSET_06, pros::E_MOTOR_ENCODER_DEGREES);
pros::MotorGroup rl({RIGHT_LOWER_BEVEL_MOTOR_1, RIGHT_LOWER_BEVEL_MOTOR_2}, pros::E_MOTOR_GEARSET_06, pros::E_MOTOR_ENCODER_DEGREES);

//Group all motors
pros::MotorGroup leftMotors({LEFT_UPPER_BEVEL_MOTOR_1, LEFT_UPPER_BEVEL_MOTOR_2, LEFT_LOWER_BEVEL_MOTOR_1, LEFT_LOWER_BEVEL_MOTOR_2}, pros::E_MOTOR_GEARSET_06, pros::E_MOTOR_ENCODER_DEGREES);
pros::MotorGroup rightMotors({RIGHT_UPPER_BEVEL_MOTOR_1, RIGHT_UPPER_BEVEL_MOTOR_2, RIGHT_LOWER_BEVEL_MOTOR_1, RIGHT_LOWER_BEVEL_MOTOR_2}, pros::E_MOTOR_GEARSET_06, pros::E_MOTOR_ENCODER_DEGREES);

pros::Rotation left_rotation_sensor(LEFT_ROTATION_SENSOR_PORT, true);
pros::Rotation right_rotation_sensor(RIGHT_ROTATION_SENSOR_PORT, true);

//Stuff
int leftX, leftY, rightX, rightY;
int left_turn_speed, right_turn_speed;
float other_angle, target_angle = 0;
bool setAngle = false;

//Joystick deadband value
const int DEADBAND = 20;

//Maximum RPM for VEX Green cartridges
const int MAX_RPM = 600;

// Scaling factor from joystick to RPM
const double SCALING_FACTOR = 600.0 / 127.0;

//PID control constants
const float lkP = 1.65;
const float lkI = 0.0;
const float lkD = 0.0;

const float rkP = 1.65;
const float rkI = 0.0;
const float rkD = 0.0;

//PID variables
double left_target_angle = 0.0;
double right_target_angle = 0.0;
double left_integral = 0.0;
double right_integral = 0.0;
double left_prev_error = 0.0;
double right_prev_error = 0.0;