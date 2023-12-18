#include "main.h"

using namespace pros;

/*****************************************
 * 
 * 
 * 
 *   SETUP
 * 
 * 
 * 
******************************************/

Motor FL(5, E_MOTOR_GEARSET_06, 1);
Motor BL(3, E_MOTOR_GEARSET_06, 1);
Motor TL(6, E_MOTOR_GEARSET_06, 0);
Motor FR(11, E_MOTOR_GEARSET_06, 0);
Motor BR(12, E_MOTOR_GEARSET_06, 0);
Motor TR(13, E_MOTOR_GEARSET_06, 1);

Motor INT(1, E_MOTOR_GEARSET_06, 1);
//Motor CL(18, E_MOTOR_GEARSET_18, 1);
Motor CR(20, E_MOTOR_GEARSET_18, 0);

MotorGroup leftMotors({BL, TL});
MotorGroup rightMotors({BR, TR});

ADIDigitalOut rightWing('A');
ADIDigitalOut leftWing('B');
ADIDigitalOut blocker('C');
ADILED autoPuncherIndicator('H',1);

bool blockerUp = false;

Controller Controller1(CONTROLLER_MASTER);
Controller Controller2(CONTROLLER_PARTNER);

Optical OPT1(4);
Optical OPT2(9);
//ADIEncoder ENC('C', 'D', false);

Imu Inr(8);

int triballsFired = 0;


lemlib::Drivetrain_t drivetrain {
	&leftMotors,
	&rightMotors,
	10, // distance between front wheels
	3.25, // wheel diameter
	360, // wheel rpm (gearing * motor rpm)
	2 // chasePower
};

// forward/backward PID
lemlib::ChassisController_t lateralController {
    14, // kP
    48, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    8 // slew rate
};
 
// turning PID
lemlib::ChassisController_t angularController {
    6, // kP
    40, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    2 // slew rate
};

// odometry struct
lemlib::OdomSensors_t sensors {
	nullptr, // vertical tracking wheel 1
	nullptr, // vertical tracking wheel 2
	nullptr, // horizontall tracking wheel 1 (add later)
    nullptr, // horizontal tracking wheel 2
    &Inr // inertial sensor
};

// create the chassis
lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);

/*****************************************
 * 
 * 
 * 
 *   CUSTOM FUNCTIONS
 * 
 * 
 * 
******************************************/

void overheatWarning(Motor motor) {
    if (motor.get_port() > 0 && motor.get_temperature() >= 74) {
        Controller1.set_text(2, 4, "OVERHEAT (" + std::to_string(motor.get_port()) + ", " + std::to_string(motor.get_temperature()) + ")");
    }
}

bool autoFireOn;

bool triballOnKicker() {
	//if detect triball green color on kicker
    double hue1 = OPT1.get_hue();
    if (hue1 > 80 && hue1 < 95 && OPT1.get_proximity() > 250) {
        return true;
    }
    return false;
}

void readyCata() {
	Controller1.print(1,1, "CATA READYING ");
	while (OPT2.get_proximity() < 40) {
		CR.move(110);
		delay(10);
	}
	CR.move(0);
}

void fireCata(int cataSpeed = 90) {

	CR.move(cataSpeed);

	while(triballOnKicker()) {

		delay(5);
	}

	CR.move(0);

}

// need to fix
void autoPuncher() {

	while (true) {

		while (autoFireOn) {

			autoPuncherIndicator.set_all(16424566);
				
			readyCata();

			if (triballOnKicker()) {
				fireCata();
				triballsFired++;
			}

			delay(10);
		}

	autoPuncherIndicator.clear_all();

	delay(20);

	}
}

void screenDisplay1() {
    while (true) {
        lemlib::Pose pose = chassis.getPose(); 
        pros::lcd::print(0, "x: %f", pose.x); 
        pros::lcd::print(1, "y: %f", pose.y); 
        pros::lcd::print(2, "heading: %f", pose.theta); 
        pros::delay(20);
    }
}

void screenDisplay2() {
    while (true) {
        pros::lcd::print(0, "hue: %f", OPT2.get_hue()); 
		pros::lcd::print(1, "distance: %d", OPT2.get_proximity()); 
		pros::delay(20);
    }
}

/*****************************************
 * 
 * 
 * 
 *   AUTONOMOUS AND DRIVER CONTROL
 * 	 (Use https://path.jerryio.com/ ) - width: , length: 
 * 
 * 
 * 
******************************************/

void nearsideRisky() {
	chassis.setPose(-44,-59, 135);
	chassis.moveTo(-56, -48, 135, 1000, false, false, 0, 0, 50);
	chassis.moveTo(-53, -53, 135, 1000, false, true, 0, 0, 50);
	rightWing.set_value(1);
	delay(200);
	chassis.turnTo(0, 0, 1000);
	chassis.turnTo(-60, 0, 2000);
	rightWing.set_value(0);
	delay(200);

	INT.move(-127); // outtake
	delay(300);
	chassis.turnTo(-60, -22, 1000, false, true, 50);
	chassis.moveTo(-60, -24, 180, 1000, false, false, 5.0);
	chassis.moveTo(-60, -48, 180, 1000);

	chassis.turnTo(-28, -28, 1000);
	chassis.moveTo(-28, -28, 45, 2000);

	INT.move(127); // intake
	chassis.moveTo(-28, -3, 0, 2000);
	chassis.moveTo(-28, -8, 0, 2000, false, false);
	chassis.turnTo(40, -8, 2000);

	INT.move(-127); // outtake
	leftWing.set_value(1);
	rightWing.set_value(1);
	chassis.moveTo(-8, -8, 90, 1000, false, true, 0, 0.6, 40);
	rightWing.set_value(0);
	chassis.moveTo(-8, -40, 180, 1500, false, true, 5.0);
	INT.move(0);
	chassis.turnTo(-23, -63, 1000, false, false, 20);
}

void nearsideSafe() {
	autoFireOn = true;
	chassis.setPose(-46,-55, 135);
	chassis.moveTo(-56, -48, 145, 1000, false, false, 0, 0, 50);
	chassis.moveTo(-53, -53, 145, 1000, false, true, 0, 0, 50);
	rightWing.set_value(1);
	delay(200);
	chassis.turnTo(0, 0, 1000);
	chassis.turnTo(-60, -40, 2000, false, true);
	rightWing.set_value(0);
	delay(200);

	chassis.moveTo(-56, -47, 155, 2000, false, false, 0);
	chassis.moveTo(-60, -24, 180, 2000, false, false, 20);
	delay(200);
	INT.move(-127);
	chassis.moveTo(-34, -60, 90, 2000);
	chassis.moveTo(-7, -59, 90, 2000);
	leftWing.set_value(1);
}

void nearsideRush() {

	autoFireOn = false;
	CR.move(120);
	delay(500);
	CR.move(0);
	autoFireOn = true;

	//rush middle and drop off alliance triball
	chassis.setPose(-36, -62, 0);
	chassis.moveTo(-30, -12, 0, 1500);
	chassis.turnTo(-47, -12, 1000);
	INT.move(-127);
	delay(400);

	chassis.turnTo(-30, -12, 1000);
	INT.move(127);
	chassis.moveTo(-30, -12, 0, 2000);
	chassis.turnTo(-4, -12, 1000);
	INT.move(-127);
	leftWing.set_value(1);
	rightWing.set_value(1);
	delay(400);
	chassis.moveTo(-4, -12, 90, 1000);
	chassis.turnTo(-45, -58, 1000);
	chassis.moveTo(-45, -58, 2000, -135);
	chassis.turnTo(-6, -59, 1000);
	INT.move(-127);
	chassis.moveTo(-5, -59, 90, 2000);
	INT.move(0);
}

void fourBall() {
	autoFireOn = false;
	CR.move(120);
	delay(500);
	CR.move(0);
	chassis.setPose(45, -60, 225);
	chassis.moveTo(57, -28, 180, 1500, false, false, 8, 0);
	chassis.moveTo(57, -49, 180, 1000);
	chassis.turnTo(7, -30, 1000);
	INT.move(127);
	autoFireOn = true;
	chassis.moveTo(7, -29, -70, 1500);
	INT.move(0);
	chassis.turnTo(17, -21, 1000);
	chassis.moveTo(17, -21, 90, 1000);
	INT.move(-127);
	delay(1500);
	INT.move(127);
	chassis.turnTo(0, -7, 1000);
	chassis.moveTo(-5, -6, -40, 1000);
	chassis.turnTo(48, -7, 1000);
	INT.move(-127);
	delay(400);
	leftWing.set_value(1);
	rightWing.set_value(1);
	chassis.moveTo(40, -9, 90, 1000, false, true, 10);
	leftWing.set_value(0);
	rightWing.set_value(0);
	chassis.moveTo(15, -25, 90, 1500, false, false);
}

// needs reworked
void fiveBallMidRush() {
	// release intake and set pose
	chassis.setPose(47, -53, 320);
	autoFireOn = true;

	// hit alliance triball toward goal with right win
	rightWing.set_value(1);

	// grab central far triball, turn and score both alliance and central far
	INT.move(127);
	chassis.turnTo(9, -5, 1000);
	chassis.moveTo(9, -5, 320, 300, false, true, 20);
	autoFireOn = false;
	rightWing.set_value(0);
	chassis.moveTo(9, -5, 320, 2500, false, true, 5);
	chassis.turnTo(40, -4, 800);
	INT.move(-127);
	chassis.turnTo(40, -4, 250);
	leftWing.set_value(1);
	rightWing.set_value(1);
	chassis.moveTo(40, -4, 90, 1300, false, true, 20);
	chassis.moveTo(32, -4, 90, 750, false, false);
	leftWing.set_value(0);
	rightWing.set_value(0);

	// grab central safe triball
	chassis.turnTo(13, -17, 1000);
	INT.move(127);
	chassis.moveTo(13, -15, 240, 1500);
	chassis.turnTo(56, -46, 500);
	chassis.moveTo(56, -46, 140, 2500);

	// go back and knock out matchload
	rightWing.set_value(1);
	INT.move(-127);
	chassis.turnTo(45, -24, 1000);
	rightWing.set_value(0);
	chassis.turnTo(60, -34, 1000, false, true);

	// tap in alliance, matchload and central safe
	chassis.moveTo(61, -30, 200, 1000, false, false);
	BL.move(-127);
	BR.move(-127);
	TL.move(-127);
	TR.move(-127);
	FL.move(-127);
	FR.move(-127);
	delay(1000);
	BL.move(20);
	BR.move(20);
	TL.move(20);
	TR.move(20);
	FL.move(20);
	FR.move(20);
	//chassis.moveTo(60, -20, 180, 1500, false, false, 100);
}

void sixBall() {
	// release the intake
	autoFireOn = true;

	// set the pose
	chassis.setPose(16, -59, -90);

	// move forward and intake the triball
	INT.move(127);
	chassis.moveTo(5, -59, -90, 1000);

	// move back and knock out the alliance triball
	chassis.moveTo(43, -59, -90, 2000, false, false, 0, 0);
	rightWing.set_value(1);
	chassis.turnTo(54, -48, 1000);
	INT.move(-127);
	chassis.moveTo(54, -48, 20, 1000, false, true, 0, 0);
	chassis.turnTo(60, 0, 1000);

	// knock in first 3 triballs
	rightWing.set_value(0);
	chassis.turnTo(62, -35, 1000);
	INT.move(-127);
	delay(500);
	chassis.turnTo(70, -24, 1000, false, true);
	chassis.moveTo(62, -36, 180, 1000, false, false);

	// get safe mid ball
	chassis.turnTo(47, -43, 1000);
	chassis.moveTo(47, -43, 220, 1000);
	INT.move(127);
	chassis.turnTo(7, -30, 1000);
	INT.move(127);
	autoFireOn = true;
	chassis.moveTo(7, -29, -70, 1500);
	INT.move(0);

	// roll safe mid ball toward goal	
	chassis.turnTo(17, -21, 1000);
	chassis.moveTo(17, -21, 90, 1000);
	INT.move(-127);
	delay(1500);
	INT.move(127);

	// get center ball
	chassis.turnTo(2, -3, 1000);
	chassis.moveTo(2, -3, -30, 1000);
	chassis.turnTo(48, -7, 1000);
	INT.move(-127);
	delay(400);

	// knock in all balls
	leftWing.set_value(1);
	rightWing.set_value(1);
	chassis.moveTo(40, -9, 90, 1000, false, true, 10);
	leftWing.set_value(0);
	rightWing.set_value(0);
	chassis.moveTo(15, -25, 0, 1500, false, false);

}

void skills() {
	autoFireOn = false;
	triballsFired = 0;
	chassis.setPose(-43, -57, 0);
	chassis.moveTo(-52, -57, -45, 1000, false, false);
	chassis.turnTo(46, -3, 1000);
	autoFireOn = true;
	while (triballsFired < 5) {
		pros::delay(20);
	}
	chassis.moveTo(48, -54, -90, 6000);
	chassis.moveTo(62, -41, 180, 1000);
	chassis.moveTo(60, -31, 0, 1000, false, false, 0.6, 0.0, 110);
	chassis.moveTo(60, -40, 0, 1000, false, true);
	chassis.moveTo(60, -31, 0, 1000, false, false, 0.6, 0.0, 110);
	chassis.moveTo(60, -40, -45, 1000, false, true);
	chassis.moveTo(13, -32, 0, 2000, false, true, 0, 0.6);
	leftWing.set_value(1);
	rightWing.set_value(1);
	chassis.moveTo(40, -4, 90, 4000, false, true, 0.6, 0.9, 127);
}

ASSET(skillspath1_txt);

void skills2() {
	chassis.follow(skillspath1_txt, 15000, 4);
	leftWing.set_value(1);
	rightWing.set_value(1);
	chassis.moveTo(40, 0, 90, 2000, false, true, 100, 0);
}

/*****************************************
 * 
 * 
 * 
 *   VEX BUILT IN CODE
 * 
 * 
 * 
******************************************/

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	chassis.calibrate();
	lcd::initialize();
    selector::init();
	OPT1.set_led_pwm(30);
	OPT2.set_led_pwm(30);
	autoFireOn = true;
	Task autoPuncherTask(autoPuncher);
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
	autoFireOn = true;

    if(selector::auton == 1){nearsideSafe();} // safe
    if(selector::auton == 2){nearsideRisky();} // risky
    if(selector::auton == 3){nearsideRush();} // rush
    if(selector::auton == -1){fourBall();} // 3 ball
    if(selector::auton == -2){fiveBallMidRush();} // 4 ball
    if(selector::auton == -3){sixBall();} // 5 ball
    if(selector::auton == 0){skills();} // skills
	
	autoFireOn = false;
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

	Task controllerScreen(screenDisplay1);

	autoFireOn = false;
	blockerUp = false;

	Controller1.set_text(1,0,"Drivecontrol started");

    // Variables
    float driveSpeed = .9;
    float turnSpeed = .4;
	bool leftWingOut = false;
	bool rightWingOut = false;
	bool cataMotorOn = false;

	//chassis.motorsStop();

	FL.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	FR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	BL.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	BR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	TL.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	TR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

	CR.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	INT.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	// !!! SWITCH TO EVENT BASED !!!
	/**
	 * BUTTON INPUT SYSTEM
	 */

	while (true) {

        // ******************************************
		// ROBOT FUNCTIONS						   //
		// ******************************************

		// ******************************************
		// CONTROLLER 1							   //
		// ******************************************

		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_A)){
			if (cataMotorOn == false) {
				CR.move(100);
				cataMotorOn = true;
			}
			else {
				CR.move(0);
				cataMotorOn = false;
			}
		}

		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_B)) {
			if (autoFireOn) {
				autoFireOn = false;
			}
			else {
				autoFireOn = true;
				Controller1.set_text(1, 1, "AUTOFIREON == true");
			}
		}

		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_X)) {
			if (blockerUp == false) {
				blockerUp = true;
				blocker.set_value(1);
			}
			else {
				blockerUp = false;
				blocker.set_value(0);
			}
		}

		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_DOWN)) {
			autonomous();
		}

		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_LEFT)) {
			chassis.setPose(0,0,0);
			chassis.turnTo(1, 0, 1000);
		}

		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_RIGHT)) {
			chassis.setPose(0,0,0);
			chassis.moveTo(0, 10, 0, 2000);
		}

		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_UP)) {
			selector::auton++;
		}

		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_L2)){
			if (leftWingOut == false) {
				leftWing.set_value(1);
				leftWingOut = true;
			}
			else {
				leftWing.set_value(0);
				leftWingOut = false;
			}
		}

		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_L1)){
			if (rightWingOut == false) {
				rightWing.set_value(1);
				rightWingOut = true;
			}
			else {
				rightWing.set_value(0);
				rightWingOut = false;
			}
		}

		if (Controller1.get_digital(E_CONTROLLER_DIGITAL_R2)){
			INT.move(-127);
		}
		else if (Controller1.get_digital(E_CONTROLLER_DIGITAL_R1)){
			INT.move(127);
		} 
		else {
			INT.move(0);
		}

		// Simple linear drive controls, based on the left and right sides and based on the analog system out of 127 multiplied by the RPM of the drives
		/**double drive = Controller1.get_analog(ANALOG_LEFT_Y);

		double turn = Controller1.get_analog(ANALOG_RIGHT_X);

		double left = (((drive * driveSpeed + turn * turnSpeed)) / 127 * 600);

		double right = (((drive * driveSpeed - turn * turnSpeed)) / 127 * 600);

		if (left > 600) {left = 600;} else if (left < -600) {left = -600;}
		if (right > 600) {right = 600;} else if (right < -600) {right = -600;}
		
		FL.move_velocity(left);
		TL.move_velocity(left);
		BL.move_velocity(left);
		FR.move_velocity(right);
		TR.move_velocity(right);
		BR.move_velocity(right);   
		**/

		// Lem drive control, basically the same as above with a function controller to add curve to drive, using the equation y=ax^{3}+(1-a)x, where a is the changed variable
		chassis.arcade(Controller1.get_analog(ANALOG_LEFT_Y), Controller1.get_analog(ANALOG_RIGHT_X), 0);
		
		overheatWarning(FL);
        overheatWarning(TL);
        overheatWarning(BL);
        overheatWarning(FR);
        overheatWarning(TR);
        overheatWarning(BR);
        overheatWarning(CR);
        overheatWarning(INT);

		pros::delay(20);
	}
}

