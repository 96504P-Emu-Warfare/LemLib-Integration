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

Motor FL(3, E_MOTOR_GEARSET_06, 1);
Motor FR(6, E_MOTOR_GEARSET_06, 0);
Motor BL(10, E_MOTOR_GEARSET_06, 1);
Motor BR(5, E_MOTOR_GEARSET_06, 0);
Motor TL(8, E_MOTOR_GEARSET_06, 0);
Motor TR(4, E_MOTOR_GEARSET_06, 1);

Motor INT(17, E_MOTOR_GEARSET_18, 1);
Motor CL(18, E_MOTOR_GEARSET_18, 1);
Motor CR(19, E_MOTOR_GEARSET_18, 1);

MotorGroup leftMotors({FL, BL, TL});
MotorGroup rightMotors({FR, BR, TR});

ADIDigitalOut rightWing('A');
ADIDigitalOut leftWing('B');

Controller Controller1(CONTROLLER_MASTER);
Controller Controller2(CONTROLLER_PARTNER);

Optical OPT1(12);
Optical OPT2(13);
ADIEncoder ENC('C', 'D', false);

Imu Inr(16);

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
    8, // kP
    30, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    5 // slew rate
};
 
// turning PID
lemlib::ChassisController_t angularController {
    4, // kP
    40, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    3 // slew rate
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
        Controller1.set_text(2, 8, "OVERHEAT");
        Controller1.rumble("......");
    }
}

bool autoFireOn;

bool triballOnKicker() {
	//if detect triball green color on kicker
    double hue1 = OPT1.get_hue();
    if (hue1 > 80 && hue1 < 100) {
        return true;
    }
    return false;
}

bool triballInCata() {
	//if detect triball green color in cata
	double hue2 = OPT2.get_hue();
	if (hue2 > 80 && hue2 < 100) {
        return true;
    }
    return false;
}

void readyCata() {
	if (OPT2.get_proximity() < 230) {
		CR.move_velocity(70);
		Controller1.set_text(0,0, "Cata readying    ");
		delay(20);
	}
	else {
		CR.move_velocity(0);
		Controller1.set_text(0,0, "Cata ready       ");
	}
}

void fireCata(int cataSpeed = 80) {
	if (OPT2.get_proximity() > 200) {
		CR.move_velocity(cataSpeed);
		Controller1.set_text(0,0, "Cata firing      ");
		delay(20);
	}
	else {
		CR.move_velocity(0);
		Controller1.set_text(0,0, "Cata fired      ");
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
 * 	 (Use https://path.jerryio.com/ )
 * 
 * 
 * 
******************************************/

void blueSixBall() {
    return;
}

void redNearsideWPRisky() {
	chassis.setPose(-44,-59, -45);
	leftWing.set_value(1);
	chassis.moveTo(-56, -50, 0, 2000);
	leftWing.set_value(0);
	chassis.turnTo(-18, -16, 1000);
	chassis.moveTo(-24, -12, -90, 4000);
	chassis.moveTo(-43, -12, -90, 2000);
	INT.move_velocity(200);
	chassis.moveTo(26, -3, 80, 3000);
	INT.move_velocity(-200);
	chassis.moveTo(-1, -7, 90, 3000);
	INT.move_velocity(200);
	leftWing.set_value(1);
	rightWing.set_value(1);
	chassis.moveTo(-5, -46, 180, 4000);
}

void skills() {
	int triballs = 0;
	chassis.setPose(-43, -57, -90);
	chassis.moveTo(-52, -57, 45, 1000);
	chassis.turnTo(46, -3, 1000);
	while (triballs < 44) {
		readyCata();
		fireCata(100);
		triballs++;
	}
	chassis.moveTo(48, -54, 90, 6000);
	chassis.moveTo(62, -41, 0, 1000);
	leftWing.set_value(1);
	rightWing.set_value(1);
	chassis.moveTo(60, -31, 0, 1000, false, true, 0.8);
	chassis.moveTo(60, -40, 0, 1000, false, false);
	chassis.moveTo(60, -31, 0, 1000, false, true, 0.8);

}

ASSET(skillspath1_txt);

void skills2() {
	chassis.follow(skillspath1_txt, 15000, 10);
	leftWing.set_value(1);
	rightWing.set_value(1);
	chassis.moveTo(40, 0, 90, 2000, false, true, 100, 0);
}

void autoCata() {
	while (autoFireOn) {
			
		readyCata();

		if (triballInCata() || triballOnKicker()) {
			fireCata();
		}

		delay(100);
	}
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

    if(selector::auton == 1){} // Red 1
    if(selector::auton == 1){} // Red 2
    if(selector::auton == 1){redNearsideWPRisky();} // Red 3
    if(selector::auton == 1){} // Blue 1
    if(selector::auton == 1){} // Blue 2
    if(selector::auton == 1){} // Blue 3
    if(selector::auton == 1){skills();} // Skills
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

	Task controllerScreen(screenDisplay2);

	autoFireOn = true;

	readyCata();

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

	CL.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	CR.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	/**
	 * BUTTON INPUT SYSTEM
	 */
	

	while (true) {

        // ******************************************
		// ROBOT FUNCTIONS						   //
		// ******************************************
        
		Task autoCataTask(autoCata);


		// ******************************************
		// CONTROLLER 1							   //
		// ******************************************


		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_A)){
			if (cataMotorOn == false) {
				CL.move_velocity(90);
				cataMotorOn = true;
			}
			else {
				CL.move_velocity(0);
				cataMotorOn = false;
			}
		}

		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_B)) {
			if (autoFireOn == true) {
				autoFireOn = false;
			}
			else {autoFireOn = true;}
		}

		// For tuning lateral PID
		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_LEFT)) {
			chassis.setPose(0,0,0);
			chassis.moveTo(10, 0, 0, 5000);
		}

		// For tuning angular PID
		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_RIGHT)) {
			chassis.setPose(0,0,0);
			chassis.turnTo(30, 0, 5000);
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
			INT.move_velocity(-127);
		}
		else if (Controller1.get_digital(E_CONTROLLER_DIGITAL_R1)){
			INT.move(127);
		} 
		else {
			INT.move(0);
		}

		// Simple linear drive controls, based on the left and right sides and based on the analog system out of 127 multiplied by the RPM of the drives
		double drive = Controller1.get_analog(ANALOG_LEFT_Y);

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
		/**
		overheatWarning(FL);
        overheatWarning(TL);
        overheatWarning(BL);
        overheatWarning(FR);
        overheatWarning(TR);
        overheatWarning(BR);
        overheatWarning(CL);
        overheatWarning(CR);
        overheatWarning(INT);
        */

		pros::delay(20);

	}
}

