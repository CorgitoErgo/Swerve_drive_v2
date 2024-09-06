#include "main.h"

//Deadband to joystick inputs
double apply_deadband(double value){
    return (fabs(value) > DEADBAND) ? value : 0.0;
}

//Limit values within the max RPM
double bound_value(double value){
    if (value > MAX_RPM) return MAX_RPM;
    if (value < -MAX_RPM) return -MAX_RPM;
    return value;
}

void set_wheel_angle(bool left_motor, pros::Motor &top_motor, pros::Motor &bottom_motor, pros::Rotation &rotation_sensor, int target_angle){
    int current_angle = getNormalizedSensorAngle(rotation_sensor.get_position());
    int error = target_angle - current_angle;

	if (error > 180) error -= 360;
    if (error < -180) error += 360;

	if(left_motor == true){
		double derivative = error - left_prev_error;
		left_integral += error;
		double output = lkP * error + lkI * left_integral + lkD * derivative;

		if(abs(error) < 2) output = 0;
		else output = bound_value(output);

		top_motor.move_velocity(output);
		bottom_motor.move_velocity(-output);

		left_prev_error = error;
	}
	else{
		double derivative = error - right_prev_error;
		right_integral += error;
		double output = rkP * error + rkI * right_integral + rkD * derivative;

		if(abs(error) < 2) output = 0;
		else output = bound_value(output);

		top_motor.move_velocity(output);
		bottom_motor.move_velocity(-output);

		right_prev_error = error;
	}
}

void reset_wheels_to_forward(){
    double target_angle = 90;
    double allowed_error = 2.0;

    while(true){
        double left_current_angle = getNormalizedSensorAngle(left_rotation_sensor.get_position());
        double right_current_angle = getNormalizedSensorAngle(right_rotation_sensor.get_position());

        double left_error = fabs(left_current_angle - target_angle);
        double right_error = fabs(right_current_angle - target_angle);

        if (left_error <= allowed_error && right_error <= allowed_error) {
            break;
        }

        set_wheel_angle(true, lu, ll, left_rotation_sensor, target_angle);
        set_wheel_angle(false, ru, rl, right_rotation_sensor, target_angle);

        pros::delay(5);
    }

    brake();
	left_target_angle = 0.0;
	right_target_angle = 0.0;
	left_integral = 0.0;
	right_integral = 0.0;
	left_prev_error = 0.0;
	right_prev_error = 0.0;
}

double translation_x = 0;
double translation_y = 0;
double rotation = 0;
bool is_power_reversed = false;

void update_wheel_power(){
	while(true){
		translation_x = apply_deadband(translation_x);
		translation_y = apply_deadband(translation_y);
		rotation = apply_deadband(rotation);

		translation_x *= SCALING_FACTOR;
		translation_y *= SCALING_FACTOR;
		rotation *= SCALING_FACTOR;

		double translation_magnitude = sqrt(translation_x * translation_x + translation_y * translation_y);
		double translation_angle = atan2(translation_y, translation_x) * (180.0 / M_PI);

		if (translation_angle < 0) translation_angle += 360;

		double left_current_angle = left_rotation_sensor.get_position();
		double right_current_angle = right_rotation_sensor.get_position();

		double left_target_angle = translation_angle + (rotation != 0 ? 45 : 0);
		double right_target_angle = translation_angle - (rotation != 0 ? 45 : 0);

		if (left_target_angle >= 360) left_target_angle -= 360;
        if (right_target_angle >= 360) right_target_angle -= 360;
        if (left_target_angle < 0) left_target_angle += 360;
        if (right_target_angle < 0) right_target_angle += 360;

		double left_angle_diff = fmod(fabs(left_target_angle - left_current_angle), 360.0);
		double right_angle_diff = fmod(fabs(right_target_angle - right_current_angle), 360.0);

		if (left_angle_diff > 180) left_angle_diff = 360 - left_angle_diff;
		if (right_angle_diff > 180) right_angle_diff = 360 - right_angle_diff;

		bool reverse_power = (left_angle_diff > 179.0 || right_angle_diff > 179.0);

		if(reverse_power){
            if(!is_power_reversed){
                //Apply power reversal
                is_power_reversed = true;
            }
        } 
		else{
            if(is_power_reversed){
                //Power should no longer be reversed
                is_power_reversed = false;
            }
        }

		if(!is_power_reversed){
            set_wheel_angle(true, lu, ll, left_rotation_sensor, left_target_angle);
            set_wheel_angle(false, ru, rl, right_rotation_sensor, right_target_angle);
        }

		double left_power = translation_magnitude + rotation;
		double right_power = translation_magnitude - rotation;

		if(is_power_reversed){
			left_power = -left_power;
			right_power = -right_power;
		}

		left_power = bound_value(left_power);
		right_power = bound_value(right_power);

		lu.move(left_power);
		ll.move(left_power);
		ru.move(right_power);
		rl.move(right_power);

		pros::delay(10);
	}
}

void initialize(){
	pros::lcd::clear();

	leftMotors.set_brake_mode(MOTOR_BRAKE_BRAKE);
	rightMotors.set_brake_mode(MOTOR_BRAKE_BRAKE);

	left_rotation_sensor.set_data_rate(5);
    right_rotation_sensor.set_data_rate(5);

	reset_wheels_to_forward();

	left_rotation_sensor.set_position(90);
    right_rotation_sensor.set_position(90);

	pros::Task translate(update_wheel_power);
}

void opcontrol(){
	while(true){
        double left_joystick_x = master.get_analog(ANALOG_LEFT_X);
        double left_joystick_y = master.get_analog(ANALOG_LEFT_Y);
        double right_joystick_x = master.get_analog(ANALOG_RIGHT_X);

        translation_x = apply_deadband(left_joystick_x);
        translation_y = apply_deadband(left_joystick_y);
        rotation = apply_deadband(right_joystick_x);

        pros::delay(15);
    }
}