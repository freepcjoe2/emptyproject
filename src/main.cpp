//BEFORE CODING
//display every step on the lcd for debugging purposes
//use the pros::lcd::print function to print the current step of the code
//If it didn't work, try this will help you debug your code and find out where the problem is

//to know:
//if the robotx and roboty updates functionably

#include "main.h"
#include "liblvgl/llemu.h"
#include "pros/screen.hpp"
#include "pros/colors.hpp"
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <vector>
#include "pros/link.hpp"
/*No this is not working
void watchdogInit () {}
void initializeIO() {
  watchdogInit();
}*/

//everything should be tuned be HERE
//tuning should based on #define-ed values
#define leftMotorPort {3, 12, -13}
#define rightMotorPort {15, -16, 17}
#define motorIntakePort 18
#define motorArmPort 20
#define wingPort 14
#define forwardOdomPort 4
#define imuPort 1
#define rightMotorOffset 0.81
#define pistonPort 'G'
#define PI 3.141592653589793
#define TRACK_WHEEL_DIAMETER_IN 2.75
//requred to change the angle of the wings for expansion
#define wingAutoCtrl false  // to false to disable automatic wing control and control the wings manually with the controller
#define wingStateOneAngle 90
#define wingStateTwoAngle 0
#define wingStateThreeAngle 45
//not measured yet

//for odometry
double robotX = 0.0;
double robotY = 0.0;
double robotHeadingDeg = 0.0;
double prevDeg = 0.0;


const double DIST_PER_DEG =
    (TRACK_WHEEL_DIAMETER_IN * PI) / 360.0;

// PROS Rotation 返回 centidegrees（1/100 度），需转换为度
const double CENTIDEG_TO_DEG = 1.0 / 100.0;

// -------------------------------
// ARM CONTROL
// -------------------------------
#define ARM_UP_DURATION   25
#define ARM_KEEP_DURATION 10
#define ARM_DOWN_DURATION 25
enum ArmState {
    UP,
    KEEP,
    DOWN,
    IDLE
};
enum WingState {
	WING_STATE_ONE,
	WING_STATE_TWO,
	WING_STATE_THREE
};
WingState wingState = WING_STATE_ONE;
ArmState armState = IDLE;
int armCounter = 0;



int tickMainWhile = 0; // For testing weak function linking 1tick=20ms

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Controller partner(pros::E_CONTROLLER_PARTNER);
// -------------------------------
// DRIVE MOTOR GROUPS
// -------------------------------
pros::MotorGroup left_motors(leftMotorPort);//left drive motors
pros::MotorGroup right_motors(rightMotorPort);//right drive motors
//pros::MotorGroup drivetrain({3, -11, 12, 15, -16, 17});//is this meanful?
// -------------------------------
// MECHANISM MOTORS
// -------------------------------
pros::Motor motorIntake(motorIntakePort);// Intake motor
pros::Motor motorArm(motorArmPort);// Arm/arm motor
pros::Motor wing(wingPort);// Wing for expansion
// -------------------------------
// ODOMETRY SENSORS
// -------------------------------
pros::Rotation forwardOdom(forwardOdomPort);   // tracking wheel
pros::Imu imu(imuPort);                // IMU
// -------------------------------
// lift
// -------------------------------
pros::adi::Pneumatics lift_piston(pistonPort, false, false);
	
bool FUNCTION_AS_PREDICTED = true; // For testing weak function linking
int ERROR_CODE = 0; // For testing weak function linking
//CODE FOR TESTING WEAK FUNCTION LINKING
//ERROR_CODE = 1; // Controller not connected
//ERROR_CODE = 2; // Controller connected to partner

// -------------------------------
// ODOMETRY UPDATE
// -------------------------------
void updateOdometry(pros::Rotation forwardOdom, pros::Imu imu) {
    double currDeg = forwardOdom.get_position() * CENTIDEG_TO_DEG;
    double dDeg = currDeg - prevDeg;
    prevDeg = currDeg;
    double dInches = dDeg * DIST_PER_DEG;
    robotHeadingDeg = imu.get_rotation();
    double headingRad = robotHeadingDeg * (PI / 180.0);
    robotX += dInches * cos(headingRad);
    robotY += dInches * sin(headingRad);
}
// -------------------------------
// DRIVE STRAIGHT
// -------------------------------

void driveDistanceForward(double inches) {
    forwardOdom.reset_position();
    while (true) {
		updateOdometry(forwardOdom, imu);	
        pros::delay(20);
    }
 
    left_motors.move(0);
    right_motors.move(0);
    pros::lcd::set_text(0, "Drv OK");
}

// -------------------------------
// TURN TO ANGLE 
// -------------------------------
void turnToAngle(double targetDeg) {
	imu.reset_rotation();
    while (true) {
        updateOdometry(forwardOdom, imu);
        pros::delay(20);
    }

    left_motors.move(0);
    right_motors.move(0);
    pros::lcd::set_text(0, "Turn OK");
}

struct PID{
	double Kp, Ki, Kd;
	double integral = 0;
	double lastError = 0;
	double lastTime = 0;
	bool started = false;

	PID(double kp, double ki, double kd) : Kp(kp), Ki(ki), Kd(kd) {}

    double update(double target, double current, double now) {
        double error = target - current;
        double dt = now - lastTime;
        if (dt <= 0) dt = 0.02;	// 20ms
        if (!started) {
            started = true;
            lastTime = now;
            lastError = error;
            return std::clamp(Kp * error, -127.0, 127.0);
        }

      integral += error * dt;
        double derivative = (error - lastError) / dt;

        lastError = error;
        lastTime = now;

        double out = Kp * error + Ki * integral + Kd * derivative;
        return std::clamp(out, -127.0, 127.0);
    }

	void reset() { integral = 0; lastError = 0; started = false; }
};
//The PID struct is still testing

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(0, "Version: 0.3.0");
	pros::lcd::set_text(1, "THIS IS A TEST MESSAGE");
	pros::lcd::set_text(2, "Error code: 0");
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

	forwardOdom.reset_position();

	
	const double target = 500.0;  
	const uint32_t MAX_AUTO_TIME_MS = 5000;
	const uint32_t start_time = pros::millis();
	PID drivePid(0.4, 0.02, 0.02); 
	drivePid.reset();

	while (FUNCTION_AS_PREDICTED) {
		double now = pros::millis() / 1000.0;
		double current = forwardOdom.get_position();
		double out = drivePid.update(target, current, now);

		
		left_motors.move((int)out);
		right_motors.move((int)out);
		
		pros::lcd::print(2, "Err: %.1f cur: %.0f", drivePid.lastError, current);
		pros::lcd::print(3, "out: %.0f", out);

		
		if (fabs(drivePid.lastError) < 5.0) {
			left_motors.move(0);
			right_motors.move(0);
			pros::lcd::print(4, "Done");
			break;
		}
		if (pros::millis() - start_time > MAX_AUTO_TIME_MS) {
			left_motors.move(0);
			right_motors.move(0);
			pros::lcd::print(4, "Timeout");
			break;
		}

		pros::delay(20);
	}
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
	bool tubeExtended = false;
	//copy the initialization to where u use the motor or group

	while (FUNCTION_AS_PREDICTED) {
		// -------------------------------
		// CONTROLLER DEBUGGING
		// -------------------------------
		bool partner_pushed = partner.get_digital(pros::E_CONTROLLER_DIGITAL_A); // Checks if the A button on the partner controller is pressed for debugging purposes
		if (partner_pushed) {
			FUNCTION_AS_PREDICTED = false; // Set the flag to false to stop the loop and end the program for debugging purposes
			ERROR_CODE = 1; // Set the error code to 1 for controller not connected for debugging purposes
		}
		// -------------------------------
		// DRIVE CONTROL
		// -------------------------------
		int Left_move_control = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int right_move_control = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);  // Gets the turn left/right from right joystick
		const int DEAD_BAND = 8;
		if (std::abs(Left_move_control) < DEAD_BAND) Left_move_control = 0;
		if (std::abs(right_move_control) < DEAD_BAND) right_move_control = 0;

		left_motors.move(Left_move_control);                      // Sets left motor voltage
		right_motors.move(right_move_control*rightMotorOffset);                     // Sets right motor voltage

		
		// -------------------------------
		// ARM/PUSH CONTROL
		// -------------------------------
		/*//renamed arm to push for better understanding, the name "push" is only used in this code and you can still calll it arm
		bool push_up = master.get_digital(pros::E_CONTROLLER_DIGITAL_X); // Gets whether L1 is pressed for pushing up the arm
		bool push_down = master.get_digital(pros::E_CONTROLLER_DIGITAL_Y);

		if (push_up) {
			motorArm.move(-127); // Moves the arm up at full speed
		} else if (push_down) {
			motorArm.move(64); // Moves the arm down at half speed
		} else {
			motorArm.move(0); // Stops the arm if neither button is pressed
		}*/
		
		// -------------------------------
        // ARM AUTO TRIGGER
        // -------------------------------
        if (master.get_digital_new_press(
                pros::E_CONTROLLER_DIGITAL_UP)) {
            armState = UP;
            armCounter = 0;
        }

        // -------------------------------
        // ARM CONTROL
        // -------------------------------
        armCounter++;

        switch (armState) {
            case UP:
                motorArm.move(-127);
                if (armCounter > ARM_UP_DURATION) {
                    armState = KEEP;
                    armCounter = 0;
                }
                break;

            case KEEP:
                motorArm.move(-15);
                if (armCounter > ARM_KEEP_DURATION) {
                    armState = DOWN;
                    armCounter = 0;
                }
                break;

            case DOWN:
                motorArm.move(64);
                if (armCounter > ARM_DOWN_DURATION) {
                    armState = IDLE;
                    armCounter = 0;
                }
                break;

            case IDLE:
                if (master.get_digital(
                        pros::E_CONTROLLER_DIGITAL_X))
                    motorArm.move(-127);
                else if (master.get_digital(
                             pros::E_CONTROLLER_DIGITAL_Y) ||
                         master.get_digital(
                             pros::E_CONTROLLER_DIGITAL_DOWN))
                    motorArm.move(64);
                else
                    motorArm.move(0);
                break;
        }
		if(wingAutoCtrl){
		if (master.get_digital_new_press(
                pros::E_CONTROLLER_DIGITAL_LEFT)) {
			switch (wingState)
			{
			case WING_STATE_ONE:
				wingState = WING_STATE_TWO;
				break;
			case WING_STATE_TWO:
				wingState = WING_STATE_THREE;
				break;
			case WING_STATE_THREE:
				wingState = WING_STATE_ONE;
				break;
			}
        }
		switch (wingState)
		{
		case WING_STATE_ONE:
			if(wing.get_position() < wingStateOneAngle) {
				wing.move(48);
			} else {
				wing.move(-48);
			}
			break;
		case WING_STATE_TWO:
			if(wing.get_position() > wingStateTwoAngle) {
				wing.move(-48);
			} else {
				wing.move(48);
			}
			break;
		case WING_STATE_THREE:
			if(wing.get_position() < wingStateThreeAngle) {
				wing.move(48);
			} else {
				wing.move(-48);
			}
			break;
		}
		}
		pros::lcd::print(5, "Wing Angle: %d", wing.get_position()); // Prints the current angle of the wing to the LCD for debugging purposes

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { // Checks if A is pressed for moving the intake forward
			motorIntake.move(127); // Moves the intake forward at full speed
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) { // Checks if B is pressed for moving the intake in reverse
			motorIntake.move(-127); // Moves the intake in reverse at full speed
		} else {
			motorIntake.move(0); // Stops the intake if neither button is pressed
		}

		//-------------------------------r
		// WING CONTROL
		//-------------------------------
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) { // Checks if R2 is pressed for opening the wings
			wing.move(48); // Moves the wing open at 3/8 speed
		} else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B )) { // Checks if R1 is pressed for closing the wings
			wing.move(-48); // Moves the wing closed at 3/8 speed
		}else {
			wing.move(0); // Stops the wing if neither button is pressed
		}
		
		int wing_angle = wing.get_position(); // Gets the current angle of the wing for debugging purposes

		if (master.get_digital_new_press(
                pros::E_CONTROLLER_DIGITAL_L1)) {
            tubeExtended = !tubeExtended;
        	lift_piston.set_value(tubeExtended);
        }

		

		/*
		pros::lcd::print(3, "Left: %d Right: %d ",Left_move_control, right_move_control);// Prints the joystick values to the LCD for debugging purposes
		//pros::lcd::print(5, "Speed Left Motors: %d\n Speed Right Motors: %d", left_motors.get_actual_velocity(), right_motors.get_actual_velocity()); // Prints the current speed of the motors to the LCD for debugging purposes
		*/
		
		pros::lcd::print(3, "ticks: %d", tickMainWhile); // Prints the number of times the main while loop has run for debugging purposes
		

		tickMainWhile++; // Increments the number of times the main while loop has run for debugging purposes


		updateOdometry(forwardOdom, imu); // Updates the odometry values
		int intRobotX = (int)robotX; // Converts the robot's X position to an integer for debugging purposes
		int intRobotY = (int)robotY; // Converts the robot's Y position to an integer for debugging purposes
		int intRobotHeading = (int)robotHeadingDeg; // Converts the robot's heading to an integer for debugging purposes
		pros::lcd::print(4, "X: %d Y: %d Heading: %d", intRobotX, intRobotY, intRobotHeading); // Prints the current odometry values to the LCD for debugging purposes



		pros::delay(20);                               // Run for 20 ms then update
	}

	if(!FUNCTION_AS_PREDICTED) {
		left_motors.move(0);
		right_motors.move(0);
		motorIntake.move(0);
		motorArm.move(0);
		wing.move(0);
		lift_piston.set_value(false);
		pros::lcd::print(2, "Error code: %d", ERROR_CODE); // Print the error code to the LCD for debugging purposes
	}
}
