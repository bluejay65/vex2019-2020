/* Assorted Links-
		Automatic PID Tuner - https://pros.cs.purdue.edu/v5/okapi/api/control/util/pid-tuner.html
*/

#include "main.h"
#include "okapi/api.hpp"
#define LEFT_WHEELS_PORT 20
#define RIGHT_WHEELS_PORT 11
#define LEFT_GRAB_ARM_PORT 9
#define RIGHT_GRAB_ARM_PORT 2
#define CONVEYOR_LEFT_PORT 1
#define CONVEYOR_RIGHT_PORT 10
#define PUSHING_ARM_PORT 12
#define SUPPORT_PORT 13

pros::Motor left_wheels (LEFT_WHEELS_PORT);
pros::Motor right_wheels (RIGHT_WHEELS_PORT, true);
pros::Motor left_grab_arm (LEFT_GRAB_ARM_PORT);
pros::Motor right_grab_arm (RIGHT_GRAB_ARM_PORT, true);
pros::Motor conveyor_left (CONVEYOR_LEFT_PORT);
pros::Motor conveyor_right (CONVEYOR_RIGHT_PORT, true);
pros::Motor pushing_arm (PUSHING_ARM_PORT);
pros::Motor support (SUPPORT_PORT);
pros::Controller master (CONTROLLER_MASTER);

const double grabkP = 0.001;    //TODO check if all of the k's are the right values
const double grabkI = 0.0000;   //TODO maybe use the auto-Pid Tuner
const double grabkD = 0.0000;

const double pushkP = 0.001;
const double pushkI = 0.0000;
const double pushkD = 0.0000;

const double supportkP = 0.001;
const double supportkI = 0.0000;
const double supportkD = 0.0000;

int set_grab_pos = 0;
double grab_pos [4] = {0.0, 50.0, 150.0, 250.0};

double push_angle [2] = {0.0, 50.0};//TODO set these to the right push_angles

auto driveController = okapi::ChassisControllerFactory::create(LEFT_WHEELS_PORT, -RIGHT_WHEELS_PORT);
auto leftGrabArmController = okapi::AsyncControllerFactory::posPID(-LEFT_GRAB_ARM_PORT, grabkP, grabkI, grabkD);
auto rightGrabArmController = okapi::AsyncControllerFactory::posPID(RIGHT_GRAB_ARM_PORT, grabkP, grabkI, grabkD);
auto supportController = okapi::AsyncControllerFactory::posPID(RIGHT_GRAB_ARM_PORT, supportkP, supportkI, supportkD);

auto pushController = okapi::AsyncControllerFactory::posPID(PUSHING_ARM_PORT, pushkP, pushkI, pushkD);

//Both changes the grab position when grab_pos is given a new value and always tries to move towards what grab_pos is set at
void change_grab_position_fn(void* param) {
  while(true) {
    leftGrabArmController.setTarget(grab_pos[set_grab_pos]);
    rightGrabArmController.setTarget(grab_pos[set_grab_pos]);
    leftGrabArmController.waitUntilSettled();
    rightGrabArmController.waitUntilSettled();

    pros::delay(20);
  }
}

void change_push_pos(double pos) {
  pushController.setTarget(pos);
  pushController.waitUntilSettled();
}

void conveyors_in() {
	conveyor_right.move(-100);
	conveyor_left.move(-100);
}
void conveyors_out() {
	conveyor_right.move(100);
	conveyor_left.move(100);
}
void conveyors_off() {
	conveyor_right.move(0);
	conveyor_left.move(0);
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

pros::Task change_grab_position (change_grab_position_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Change Grab Position");

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
void competition_initialize() {}

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
	set_grab_pos = 3;
	pros::delay(500);
	set_grab_pos = 0;
	conveyors_out();
	pros::delay(500);
	set_grab_pos = 2;
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

		pros::lcd::set_text(4, "Left: "+ std::to_string(pushing_arm.get_position()));

    //Drive trains move
    driveController.tank(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_Y));

    //Controls for grab arms
    if(master.get_digital(DIGITAL_DOWN) && set_grab_pos > 0) {
      set_grab_pos--;
    }
    else if(master.get_digital(DIGITAL_UP) && set_grab_pos < 3) {
      set_grab_pos++;
    }
		print_grab_pos();

    if(master.get_digital(DIGITAL_R2)) {
      conveyors_out();
    }
    else if(master.get_digital(DIGITAL_R1)) {
      conveyors_in();
    }
		else {
			conveyors_off();
		}

    //Controls for pushing arm
    if(master.get_digital(DIGITAL_L1)) {
      change_push_pos(push_angle[0]);
    }
    else if(master.get_digital(DIGITAL_L2)) {
      change_push_pos(push_angle[1]);
    }

		pros::delay(20);
	}
}
