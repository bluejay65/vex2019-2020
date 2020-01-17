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

const double grabkP = 0.005;    //TODO check if all of the k's are the right values
const double grabkI = 0.0000;   //TODO maybe use the auto-Pid Tuner
const double grabkD = 0.0000;

const double pushkP = 0.001;
const double pushkI = 0.0001;
const double pushkD = 0.0001;

const double supportkP = 0.001;
const double supportkI = 0.0001;
const double supportkD = 0.0001;

int grab_pos = -1;
int support_pos = -1;

double grab_height0 = 0.0;//TODO change these to the right grab_heights
double grab_height1 = 150.0;
double grab_height2 = 250.0;
double push_grab_height = 50.0;

double support_angle0 = 0.0;
double support_angle1 = 10.0;

double push_angle[2] = {0.0, 50.0};//TODO set these to the right push_angles

auto driveController = okapi::ChassisControllerFactory::create(LEFT_WHEELS_PORT, -RIGHT_WHEELS_PORT);
auto leftGrabArmController = okapi::AsyncControllerFactory::posPID(-LEFT_GRAB_ARM_PORT, grabkP, grabkI, grabkD);
auto rightGrabArmController = okapi::AsyncControllerFactory::posPID(RIGHT_GRAB_ARM_PORT, grabkP, grabkI, grabkD);
auto supportController = okapi::AsyncControllerFactory::posPID(RIGHT_GRAB_ARM_PORT, supportkP, supportkI, supportkD);

auto pushController = okapi::AsyncControllerFactory::posPID(PUSHING_ARM_PORT, pushkP, pushkI, pushkD);

//Both changes the grab position when grab_pos is given a new value and always tries to move towards what grab_pos is set at
void change_grab_position_fn(void* param) {
  double target = 0.0;
  int last_grab_pos = -1;
  while(true) {

    if(grab_pos != last_grab_pos) {
      if(grab_pos == 0) {
        target = grab_height0;
      }
      else if(grab_pos == 1) {
        target = grab_height1;
      }
      else if(grab_pos == 2) {
        target = grab_height2;
      }
      else {
        master.set_text(0, 5, "ERROR");// TODO make sure ERROR is centered and see if the text fits on the screen
      }
    }
    last_grab_pos = grab_pos;
    leftGrabArmController.setTarget(target);
    rightGrabArmController.setTarget(target);
    leftGrabArmController.waitUntilSettled();
    rightGrabArmController.waitUntilSettled();

    pros::delay(20);
  }
}

void change_push_pos(double pos) {
  pushController.setTarget(pos);
  pushController.waitUntilSettled();
}

void change_support_pos_fn(void* param) {
  while(true) {
    if(support_pos == 0) {
      supportController.setTarget(support_angle0);
    }
    else if(support_pos == 1) {
      supportController.setTarget(support_angle1);
    }
    pros::lcd::print(3, "support angle %d", support_pos);
    pros::lcd::print(1, "support angle 0 %d", support_angle0);
    pros::lcd::print(2, "support angle 1 %d", support_angle1);
    supportController.waitUntilSettled();

    pros::delay(50);
  }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize();

  pros::Task change_grab_position (change_grab_position_fn, (void*)"PROS", 9, TASK_STACK_DEPTH_DEFAULT, "Change Grab Position");
  pros::Task change_support_position (change_support_pos_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Change Support Position");
  change_grab_position.resume();

  bool a_pressed = false;
  while(a_pressed == false) {
    if(master.get_analog(ANALOG_LEFT_Y) >= 15) {
      left_grab_arm.move(master.get_analog(ANALOG_LEFT_Y));
      right_grab_arm.move(master.get_analog(ANALOG_LEFT_Y));
    }
    else if(master.get_analog(ANALOG_LEFT_Y) <= -15) {
      left_grab_arm.move(master.get_analog(ANALOG_LEFT_Y));
      right_grab_arm.move(master.get_analog(ANALOG_LEFT_Y));
    }

    if (master.get_digital(DIGITAL_A)) {
      master.set_text(0, 0, "Set 1 to grab_pos");
		  left_grab_arm.tare_position();
		  right_grab_arm.tare_position();

      a_pressed = true;
    }
	}
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

void autonomous() {}

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
	//okapi::PIDTunerFactory autoPID();
	while(true) {
		pros::lcd::set_text(4, "Left: "+ std::to_string(left_grab_arm.get_position()));
		pros::lcd::set_text(5, "Right: "+ std::to_string(right_grab_arm.get_position()));

    //Drive trains move
    driveController.tank(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_Y));

    //Controls for grab arms
    if(master.get_digital(DIGITAL_DOWN)) {
      grab_pos = 0;
      master.print(1, 0, "Grab_pos: %d", grab_pos);
    }
    else if(master.get_digital(DIGITAL_LEFT)) {
      grab_pos = 1;
      master.print(1, 0, "Grab_pos: %d", grab_pos);
    }
    else if(master.get_digital(DIGITAL_UP)) {
      grab_pos = 2;
      master.print(1, 0, "Grab_pos: %d", grab_pos);
    }

    // controls for support arm
    if(master.get_digital(DIGITAL_X)) {
      support_pos = 0;
    }
    else if(master.get_digital(DIGITAL_B)) {
      support_pos = 1;
    }

    if(master.get_digital(DIGITAL_R1) && master.get_digital(DIGITAL_R2)) {
      conveyor_left.move(0);
      conveyor_right.move(0);
    }
    else if(master.get_digital(DIGITAL_R2)) {
      conveyor_left.move(-100);
      conveyor_right.move(-100);
    }
    else if(master.get_digital(DIGITAL_R1)) {
      conveyor_left.move(100);
      conveyor_right.move(100);
    }

    //Controls for pushing arm
    if(master.get_digital(DIGITAL_L1)) {
      change_push_pos(push_angle[0]);
    }
    else if(master.get_digital(DIGITAL_L2)) {
      change_push_pos(push_angle[1]);
    }

    //Sets the zero position of the grabbing arms if all four shoulder buttons are pressed at the same time
		if (master.get_digital(DIGITAL_R1) && master.get_digital(DIGITAL_R2) && master.get_digital(DIGITAL_L1) && master.get_digital(DIGITAL_L2)) {
      master.set_text(0, 0, "Set 1 pos for grab_arm");
		  left_grab_arm.tare_position();
		  right_grab_arm.tare_position();
		}

		pros::delay(20);
	}
}
