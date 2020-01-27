/* Assorted Links-
		Automatic PID Tuner - https://pros.cs.purdue.edu/v5/okapi/api/control/util/pid-tuner.html
*/

#include "main.h"
#include "okapi/api.hpp"
#define LEFT_WHEELS_PORT 2
#define RIGHT_WHEELS_PORT 12
#define LEFT_GRAB_ARM_PORT 8
#define RIGHT_GRAB_ARM_PORT 10
#define CONVEYOR_LEFT_PORT 1
#define CONVEYOR_RIGHT_PORT 11
#define PUSHING_ARM_PORT 13

pros::Motor left_wheels (LEFT_WHEELS_PORT);
pros::Motor right_wheels (RIGHT_WHEELS_PORT, true);
pros::Motor left_grab_arm (LEFT_GRAB_ARM_PORT);
pros::Motor right_grab_arm (RIGHT_GRAB_ARM_PORT, true);
pros::Motor conveyor_left (CONVEYOR_LEFT_PORT);
pros::Motor conveyor_right (CONVEYOR_RIGHT_PORT, true);
pros::Motor pushing_arm (PUSHING_ARM_PORT);
pros::Controller master (CONTROLLER_MASTER);

const double grabkP = 0.0035;    //TODO check if all of the k's are the right values
const double grabkI = 0.00015;   //TODO maybe use the auto-Pid Tuner
const double grabkD = 0.00015;

const double pushkP = 0.0005;
const double pushkI = 0.0000;
const double pushkD = 0.0000;

int set_grab_pos = 0;
double grab_pos [4] = {20.0, 50.0, 100.0, 350.0};

int push_pos = 0;
double push_angle [2] = {0.0, 2000.0};//if you realign grab_arms set to 2750.0

int drive_speed [2] = {47, 94};
int sped_up = 1;

bool stop_conveyors = false;
bool stop_wheels = false;

auto driveController = okapi::ChassisControllerFactory::create(LEFT_WHEELS_PORT, -RIGHT_WHEELS_PORT);
auto leftGrabArmController = okapi::AsyncControllerFactory::posPID(LEFT_GRAB_ARM_PORT, grabkP, grabkI, grabkD);
auto rightGrabArmController = okapi::AsyncControllerFactory::posPID(-RIGHT_GRAB_ARM_PORT, grabkP, grabkI, grabkD);

auto pushController = okapi::AsyncControllerFactory::posPID(PUSHING_ARM_PORT, pushkP, pushkI, pushkD);

//Both changes the grab position when grab_pos is given a new value and always tries to move towards what grab_pos is set at
void change_right_grab_position_fn(void* param) {
  while(true) {
    rightGrabArmController.setTarget(grab_pos[set_grab_pos]);
    rightGrabArmController.waitUntilSettled();

    pros::delay(20);
  }
}
void change_left_grab_position_fn(void* param) {
  while(true) {
    leftGrabArmController.setTarget(grab_pos[set_grab_pos]);
    leftGrabArmController.waitUntilSettled();

    pros::delay(20);
  }
}

void conveyors_in(int speed) {
	conveyor_right.move(speed);
	conveyor_left.move(speed);
}
void conveyors_out(int speed) {
	conveyor_right.move(-speed);
	conveyor_left.move(-speed);
}
void conveyors_off() {
	conveyor_right.move(0);
	conveyor_left.move(0);
}

void change_push_pos_fn(void* param) {
  while(true) {
    pushController.setTarget(push_angle[push_pos]);
    pushController.waitUntilSettled();
  }
}

void drop_cubes() {
  push_pos = 1;
  pros::delay(500);
  stop_conveyors = true;
  stop_wheels = true;
  conveyors_out(100);
  left_wheels.move(-50);
  right_wheels.move(-50);
  pros::delay(1500);
  stop_conveyors = false;
  stop_wheels = false;
}

void print_grab_pos() {
	master.clear_line(2);

	pros::delay(50);

	if(set_grab_pos == 0){
		master.print(2, 0, "Grab Pos: _ _ _");
	}
	else if(set_grab_pos == 1){
		master.print(2, 0, "Grab Pos: 1 _ _");
	}
	else if(set_grab_pos == 2){
		master.print(2, 0, "Grab Pos: 1 2 _");
	}
	else if(set_grab_pos == 3){
		master.print(2, 0, "Grab Pos: 1 2 3");
	}
}

pros::Task change_right_grab_position (change_right_grab_position_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Change Right Grab Position");
pros::Task change_left_grab_position (change_left_grab_position_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Change Left Grab Position");

pros::Task change_push_pos (change_push_pos_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Change Push Position");

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {

}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void autonomous() {
	set_grab_pos = 2;
  conveyors_out(100);
	pros::delay(500);
  set_grab_pos = 1;
	pros::delay(4000);
	conveyors_off();
	set_grab_pos = 0;
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	while(true) {
		pros::lcd::set_text(2, "Left: "+ std::to_string(left_grab_arm.get_position()));
		pros::lcd::set_text(3, "Right: "+ std::to_string(right_grab_arm.get_position()));

		pros::lcd::set_text(4, "Pushing: "+ std::to_string(pushing_arm.get_position()));

    //normal move
    if(!stop_wheels) {
      left_wheels.move_voltage(master.get_analog(ANALOG_LEFT_Y)*drive_speed[sped_up]);
      right_wheels.move_voltage(master.get_analog(ANALOG_RIGHT_Y)*drive_speed[sped_up]);
    }
    else {
      left_wheels.move_voltage(0);
      right_wheels.move_voltage(0);
    }

    if(master.get_digital_new_press(DIGITAL_A)) {
      drop_cubes();
    }

    if(master.get_digital_new_press(DIGITAL_DOWN) && set_grab_pos > 0) {
      set_grab_pos -= 1;
      print_grab_pos();
    }
    else if(master.get_digital_new_press(DIGITAL_UP) && set_grab_pos < 3) {
      set_grab_pos += 1;
      print_grab_pos();
    }

    //fast conveyors
    if(master.get_digital(DIGITAL_R2)) {
      conveyors_out(127);
    }
    else if(master.get_digital(DIGITAL_R1)) {
      conveyors_in(127);
    }
    //slow conveyors
    else if(master.get_digital(DIGITAL_L2)) {
      conveyors_out(50);
    }
    else if(master.get_digital(DIGITAL_L1)) {
      conveyors_in(50);
    }
    else if(stop_conveyors){
      conveyors_off();
    }

    //Controls for pushing arm
    if(master.get_digital(DIGITAL_B)) {
      sped_up = 1;
      push_pos = 0;
    }
    else if(master.get_digital(DIGITAL_X)) {
      sped_up = 0;
      push_pos = 1;
    }

		pros::delay(20);
	}
}
