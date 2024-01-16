#include "main.h"
#include "display/lv_core/lv_obj.h"
#include "display/lv_objx/lv_img.h"

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
int globalCataSpeed = 90;
int cataDelay = 10; // in ms
bool autoLower;
bool competitionMode = false;

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
    31, // kP
    60, // kD
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
    if (OPT1.get_proximity() > 250) { //hue1 > 80 && hue1 < 95 && 
        return true;
    }
    return false;
}

void readyCata() {
	while (OPT2.get_proximity() < 40) {
		CR.move(110);
		delay(10);
	}
	CR.move(0);
}

void fireCata(int cataSpeed = 110) {

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
				pros::delay(cataDelay);
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

void controllerScreen() {
	while (true) {
		Controller1.set_text(1,1, "CS:" + std::to_string(globalCataSpeed) + " AF:" + std::to_string(autoFireOn) + " L:" + std::to_string(autoLower));
		delay(100);
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
	chassis.setPose(-46,-55, 135);
	chassis.moveTo(-56, -48, 145, 1000, false, false, 0, 0, 50);
	CR.move(0);
	autoFireOn = true;
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
	chassis.moveTo(-16, -58, 90, 2000);
	leftWing.set_value(1);
}

void nearsideRush() {
	chassis.setPose(34, -55, 0);
	leftWing.set_value(1);
	delay(300);
	INT.move(127);
	autoFireOn = true;
	chassis.moveTo(-25, -8, 10, 1500);
	chassis.moveTo(-42, -51, 15, 1500, false, false);
	chassis.turnTo(-57, -42, 1000, false, true);
	chassis.moveTo(-57, -42, 120, 1000, false, false);
	chassis.moveTo(-58, -33, 180, 1000);
	chassis.moveTo(-10, -61, 90, 1000);
	INT.move(-127);
	chassis.moveTo(-10, -61, 90, 2500);

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

void fiveBallMidRush() {
	// release intake and set pose
	chassis.setPose(47, -53, 320);

	// hit alliance triball toward goal with right win
	rightWing.set_value(1);

	// grab central far triball, turn and score both central and central far
	INT.move(127);
	chassis.turnTo(9, -5, 300);
	autoFireOn = true;
	chassis.moveTo(9, -5, 320, 300, false, true, 20);
	autoFireOn = false;
	rightWing.set_value(0);
	chassis.moveTo(9, -5, 320, 2500, false, true, 5);
	chassis.turnTo(40, -4, 800);
	INT.move(-127);
	chassis.turnTo(40, -4, 250);
	leftWing.set_value(1);
	rightWing.set_value(1);
	chassis.moveTo(40, -2, 90, 1300, false, true, 20);
	chassis.moveTo(32, -2, 90, 750, false, false);
	leftWing.set_value(0);
	rightWing.set_value(0);

	// grab central safe triball
	chassis.turnTo(13, -17, 1000);
	INT.move(127);
	chassis.moveTo(13, -15, 240, 1500);
	chassis.turnTo(55, -48, 500);
	chassis.moveTo(55, -48, 140, 2500);

	// go back and knock out matchload
	rightWing.set_value(1);
	INT.move(-127);
	chassis.turnTo(45, -24, 1000);
	rightWing.set_value(0);
	chassis.turnTo(60, -34, 1000, false, true);

	// tap in alliance, matchload and central safe
	//chassis.moveTo(61, -30, 200, 1000, false, false);
	BL.move(-127);
	BR.move(-127);
	TL.move(-127);
	TR.move(-127);
	FL.move(-127);
	FR.move(-127);
	delay(600);
	BL.move(20);
	BR.move(20);
	TL.move(20);
	TR.move(20);
	FL.move(20);
	FR.move(20);
}

void sixBall() {
	// release the intake
	autoFireOn = true;

	// set the pose
	chassis.setPose(16, -59, -90);

	// move forward and intake the triball
	INT.move(127);
	chassis.moveTo(5, -59, -90, 10000);

	// move back and knock out the alliance triball
	chassis.moveTo(43, -59, -90, 20000, false, false, 0, 0);
	rightWing.set_value(1);
	chassis.turnTo(51, -51, 10000);
	INT.move(-127);
	chassis.moveTo(51, -51, 45, 10000, false, true, 0, 0);
	chassis.turnTo(60, 0, 10000);

	// knock in first 3 triballs
	rightWing.set_value(0);
	chassis.turnTo(62, -35, 10000);
	INT.move(-127);
	delay(500);
	chassis.turnTo(70, -24, 10000, false, true);
	chassis.moveTo(61, -32, 180, 10000, false, false);

	// get safe mid ball
	chassis.turnTo(47, -43, 10000);
	chassis.moveTo(47, -43, 230, 10000);
	INT.move(127);
	chassis.turnTo(11, -26, 10000);
	autoFireOn = true;
	chassis.moveTo(11, -26, 295, 15000);
	INT.move(0);

	// roll safe mid ball toward goal	
	chassis.turnTo(24, -19, 10000);
	chassis.moveTo(24, -19, 60, 300);
	INT.move(-100);
	chassis.moveTo(24, -19, 60, 5000);

	// get center ball
	chassis.turnTo(11, -6, 10000);
	INT.move(100);
	chassis.moveTo(11, -6, 315, 10000);
	chassis.turnTo(39, -5, 10000);

	// knock in all balls
	leftWing.set_value(1);
	rightWing.set_value(1);
	INT.move(-100);
	BL.move(127);
	BR.move(127);
	TL.move(127);
	TR.move(127);
	FL.move(127);
	FR.move(127);
	delay(1000);
	leftWing.set_value(0);
	rightWing.set_value(0);
	BL.move(-20);
	BR.move(-20);
	TL.move(-20);
	TR.move(-20);
	FL.move(-20);
	FR.move(-20);

}

void skills() {
	// initial setup and setting cata to global speed
	autoFireOn = true;
	//CR.move(globalCataSpeed);
	chassis.setPose(-49, -56, 225);

	// turn toward goal and fire for 40 seconds
	chassis.turnTo(46, -9, 1000, false, true);
	pros::delay(40000); // however long it takes to fire all triballs

	// turn autofire on to keep cata down
	autoFireOn = true;

	// turn and move backwards to other side
	// pushing triballs along with robot
	INT.move(-127);
	chassis.turnTo(-19, -59, 1000, false, true);
	chassis.moveTo(-19, -59, 285, 1000, false, false);
	chassis.moveTo(39, -56, 270, 2000, false, false);

	// backwards push corner triballs into goal
	chassis.turnTo(61, -43, 1000, false, true);
	chassis.moveTo(61, -43, 230, 1500, false, false);
	chassis.turnTo(64, 0, 1000, false, true);
	//chassis.moveTo(61, -32, 180, 2000, false, false, 0.6, 20);
	BL.move(-90);
	BR.move(-90);
	TL.move(-90);
	TR.move(-90);
	FL.move(-90);
	FR.move(-90);
	delay(1500);
	chassis.setPose(61, -32, 180);
	BL.move(20);
	BR.move(20);
	TL.move(20);
	TR.move(20);
	FL.move(20);
	FR.move(20);
	delay(750);
	BL.move(0);
	BR.move(0);
	TL.move(0);
	TR.move(0);
	FL.move(0);
	FR.move(0);

	// move out of the corner and towards the middle,
	// deploying wings on way to collect triballs
	chassis.turnTo(11, -11, 1000);
	chassis.moveTo(11, -11, 315, 500);
	leftWing.set_value(1);
	chassis.moveTo(11, -11, 315, 250);
	leftWing.set_value(0);
	chassis.moveTo(11, -11, 315, 1500);

	// first front push 
	chassis.turnTo(48, -5, 1000);
	leftWing.set_value(1);
	rightWing.set_value(1);
	//chassis.moveTo(40, -5, 90, 1500, false, true, 20);
	BL.move(100);
	BR.move(100);
	TL.move(100);
	TR.move(100);
	FL.move(100);
	FR.move(100);
	delay(1000);
	chassis.setPose(40, -5, 90);
	BL.move(-20);
	BR.move(-20);
	TL.move(-20);
	TR.move(-20);
	FL.move(-20);
	FR.move(-20);
	delay(1000);
	BL.move(0);
	BR.move(0);
	TL.move(0);
	TR.move(0);
	FL.move(0);
	FR.move(0);
	delay(100);
	leftWing.set_value(0);
	rightWing.set_value(0);

	// back up and second push
	chassis.moveTo(11, -11, 90, 1500, false, false);
	chassis.turnTo(48, 3, 1000);
	leftWing.set_value(1);
	rightWing.set_value(1);
	BL.move(100);
	BR.move(100);
	TL.move(100);
	TR.move(100);
	FL.move(100);
	FR.move(100);
	delay(1000);
	leftWing.set_value(0);
	rightWing.set_value(0);
	BL.move(-20);
	BR.move(-20);
	TL.move(-20);
	TR.move(-20);
	FL.move(-20);
	FR.move(-20);
	delay(1000);
	BL.move(0);
	BR.move(0);
	TL.move(0);
	TR.move(0);
	FL.move(0);
	FR.move(0);
	delay(1000);

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
void competition_initialize() {
	competitionMode = true;
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
	autoFireOn = false;
	CR.move(80);

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

	Task brainScreen(screenDisplay1);
	Task controllerScreenTask(controllerScreen);

	if (competitionMode) {
		LV_IMG_DECLARE(BrainScreenIdle);
		lv_obj_t *img = lv_img_create(lv_scr_act(), NULL);
		lv_img_set_src(img, &BrainScreenIdle);
		lv_obj_align(img, NULL, LV_ALIGN_CENTER, 0, 0);
	}

	autoFireOn = true;
	blockerUp = false;
	autoLower = true;

    // Variables
    float driveSpeed = .9;
    float turnSpeed = .5;
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

	/**
	 * BUTTON INPUT SYSTEM
	 */

	while (true) {

        // ******************************************
		// ROBOT FUNCTIONS						   //
		// ******************************************

		if (autoLower == true && cataMotorOn == false && autoFireOn == false) {
			if (OPT2.get_proximity() < 40) {
				CR.move(90);
			}
			else {
				CR.move(0);
			}
		}

		// ******************************************
		// CONTROLLER 1							   //
		// ******************************************

		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_A)){
			if (cataMotorOn == false) {
				CR.move(globalCataSpeed);
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
			}
		}

		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_Y)) {
			if (autoLower) {
				autoLower = false;
			}
			else {
				autoLower = true;
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
			globalCataSpeed -= 5;
		}

		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_RIGHT)) {
			globalCataSpeed += 5;
		}

		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_UP)) {
			chassis.setPose(0,0,0);
			chassis.moveTo(0, 20, 0, 2000);
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
		double drive = Controller1.get_analog(ANALOG_LEFT_Y);

		double turn = Controller1.get_analog(ANALOG_RIGHT_X);

		double left = (((drive * driveSpeed + turn * turnSpeed)) / 127 * 12);

		double right = (((drive * driveSpeed - turn * turnSpeed)) / 127 * 12);
		
		FL.move_velocity(left);
		TL.move_velocity(left);
		BL.move_velocity(left);
		FR.move_velocity(right);
		TR.move_velocity(right);
		BR.move_velocity(right);   
		//**/

		// Lem drive control, basically the same as above with a function controller to add curve to drive, using the equation y=ax^{3}+(1-a)x, where a is the changed variable
		//chassis.arcade(Controller1.get_analog(ANALOG_LEFT_Y), Controller1.get_analog(ANALOG_RIGHT_X), 0);
		
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

