#include "main.h"
#include "pros/adi.hpp"
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <list>

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

Motor INT(17, E_MOTOR_GEARSET_06, 1);
Motor CR(20, E_MOTOR_GEARSET_18, 0);

MotorGroup leftMotors({BL, TL});
MotorGroup rightMotors({BR, TR});

ADIDigitalOut rightWing('A');
ADIDigitalOut leftWing('B');
ADIDigitalOut blocker('C');
ADILed driveLeds(1, 27);

bool blockerUp = false;
int globalCataSpeed = 90;
int cataDelay = 10; // in ms
bool autoLower;
bool competitionMode = false;

Controller Controller1(CONTROLLER_MASTER);
Controller Controller2(CONTROLLER_PARTNER);

Optical OPT1(4);
Optical OPT2(9);

Imu Inr(8);

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
    47, // kP
    70, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    8 // slew rate
};
 
// turning PID
lemlib::ChassisController_t angularController {
    7, // kP
    48, // kD
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
	nullptr, // horizontal tracking wheel 1 (add later)
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
		// print overheat statement in the form "OVERHEAT (motor, temp)"
        Controller1.set_text(2, 4, "OVERHEAT (" + std::to_string(motor.get_port()) + ", " + std::to_string(motor.get_temperature()) + ")");
    }
}

bool autoFireOn;

bool triballOnKicker() {
    return OPT1.get_proximity() > 250;
}

bool cataIsReadied() {
    return OPT2.get_proximity() > 40;
}

// spin cata until readied
void readyCata(int cataSpeed = 127) {
	CR.move(cataSpeed);
	while (!cataIsReadied()) {
		delay(10);
	}
	CR.move(0);
}

// spin cata until triball is off of robot
void fireCata(int cataSpeed = 127) {
	CR.move(cataSpeed);
	while(triballOnKicker()) {
		delay(10);
	}
	CR.move(0);
}

// if toggled on, automatically fire detected triballs in puncher and reset
void autoPuncher() {
	readyCata();
	while (true) {
		while (autoFireOn) {
			if (triballOnKicker()) {
				fireCata();
				readyCata();
			}
			delay(10);
		}
		delay(10);
	}
}

// if toggled on, automatically lower/ready cata
void autoLower() {
	while (true) {
		while (autoLower) {
			if (!cataIsReadied()) {readyCata()} // if statement not required but maybe better?
			delay(10);
		}
		delay(10);
	}
}

void screenDisplay1() {
    while (true) {
        lemlib::Pose pose = chassis.getPose(); 
        lcd::print(0, "x: %f", pose.x); 
        lcd::print(1, "y: %f", pose.y); 
        lcd::print(2, "heading: %f", pose.theta); 
        delay(10);
    }
}

void screenDisplay2() {
    while (true) {
        lcd::print(0, "hue: %f", OPT2.get_hue()); 
		lcd::print(1, "distance: %d", OPT2.get_proximity()); 
		delay(10);
    }
}

void controllerScreen() {
	while (true) {
		Controller1.set_text(1,1, "CS:" + std::to_string(globalCataSpeed) + " AF:" + std::to_string(autoFireOn) + " L:" + std::to_string(autoLower));
		delay(100);
	}
}

void driveMove(int power) {
	BL.move(power);
	BR.move(power);
	TL.move(power);
	TR.move(power);
	FL.move(power);
	FR.move(power);
}

// RGB CONTROL BELOW

void ledUpdater() {
	while(true){
		driveLeds.update();
		delay(100);
	}
}

uint32_t hexToDec(const std::string& hex) {
    std::stringstream ss;
    ss << std::hex << hex;
    uint32_t dec;
    ss >> dec;
    return dec;
}

std::string decToHex(uint32_t dec) {
    std::stringstream ss;
    ss << std::hex << std::setw(8) << std::setfill('0') << dec;
    return ss.str();
}

// this doesnt work 
std::vector<uint32_t> genGradient(std::string color1, std::string color2, int x) {
    uint32_t r1 = hexToDec(color1.substr(0, 2));
    uint32_t g1 = hexToDec(color1.substr(2, 2));
    uint32_t b1 = hexToDec(color1.substr(4, 2));

    uint32_t r2 = hexToDec(color2.substr(0, 2));
    uint32_t g2 = hexToDec(color2.substr(2, 2));
    uint32_t b2 = hexToDec(color2.substr(4, 2));

    std::vector<uint32_t> inBetweenColors;

    for (int i = 1; i <= x; ++i) {
        uint32_t r = r1 + (r2 - r1) * i / x;
        uint32_t g = g1 + (g2 - g1) * i / x;
        uint32_t b = b1 + (b2 - b1) * i / x;

        uint32_t hexColor = (r << 16) | (g << 8) | b;
        inBetweenColors.push_back(hexColor);
    }

    return inBetweenColors;
}

// tbh its probably just easier to make custom set pixel and set all functions to accomodate multiple string
// add public variables for turning on and off flow, and dif colors to flow

void flow(std::string color1, std::string color2) {

	// this doesnt work
	std::vector<uint32_t> colors = genGradient(color1, color2, driveLeds.length());
	
	while (true) {

		// loop through each pixel gets a color, update buffer, shift color matrix by 1, repeat
		for (int i = 0; i < driveLeds.length(); ++i) {
			driveLeds[i] = colors[i];
		}
		
		// shift color vector
		std::rotate(colors.begin(), colors.begin()+1, colors.end());
		
		delay(100);
	}
}

/*****************************************
 * 
 * 
 * 
 *   AUTONOMOUS AND DRIVER CONTROL
 * 	 (Use https:// path.jerryio.com/ ) - width: , length: 
 * 
 * 
 * 
******************************************/

void nearsideRisky() {
	chassis.setPose(-44,-59, 135);
	chassis.moveTo(-56, -48, 135, 1000, false, false, 0, 0, 50);
	chassis.moveTo(-53, -53, 135, 1000, false, true, 0, 0, 50);
	rightWing.set_value(true);
	delay(200);
	chassis.turnTo(0, 0, 1000);
	chassis.turnTo(-60, 0, 2000);
	rightWing.set_value(false);
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
	leftWing.set_value(true);
	rightWing.set_value(true);
	chassis.moveTo(-8, -8, 90, 1000, false, true, 0, 0.6, 40);
	rightWing.set_value(false);
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
	rightWing.set_value(true);
	delay(200);
	chassis.turnTo(0, 0, 1000);
	chassis.turnTo(-60, -40, 2000, false, true);
	rightWing.set_value(false);
	delay(200);

	chassis.moveTo(-56, -47, 155, 2000, false, false, 0);
	chassis.moveTo(-60, -24, 180, 2000, false, false, 20);
	delay(200);
	INT.move(-127);
	chassis.moveTo(-34, -60, 90, 2000);
	chassis.moveTo(-12, -55, 90, 3000);
}

void nearsideRush() {
	chassis.setPose(-45, -55, 45);

	// hit alliance triball toward goal with left wing
	rightWing.set_value(true);

	// grab central far triball
	INT.move(127);
	chassis.moveTo(-11, -8, 30, 300, false, true, 35);
	autoFireOn = true;
	rightWing.set_value(false);
	chassis.moveTo(-11, -8, 30, 2500, false, true, 35);
	delay(300);
	INT.move(60);

	// back up wtih triball and move to match load bar
	driveMove(-70);
	delay(900);
	driveMove(0);
	chassis.moveTo(-50, -46, 45, 3000, false, false);

	// send match load and central triball down alley
	chassis.turnTo(-22, -81, 1000);
	rightWing.set_value(true);
	INT.move(-127);
	delay(300);
	chassis.turnTo(-20, -56, 1000);
	rightWing.set_value(false);
	chassis.moveTo(-10, -58, 90, 2000);
	driveMove(0);
	leftWing.set_value(true);
	delay(300);
	leftWing.set_value(false);
}

void fiveBallMidrush() {
	
}

void sixBallMidrush() {
	// release intake and set pose
	chassis.setPose(47, -53, 320);

	// hit alliance triball toward goal with right win
	rightWing.set_value(true);
	leftWing.set_value(true);

	// grab central far triball, turn and score both central and central far
	INT.move(127);
	chassis.moveTo(10, -6, 320, 200, false, true, 20);
	rightWing.set_value(false);
	leftWing.set_value(false);
	CR.move(0);
	autoFireOn = true;
	chassis.moveTo(10, -6, 320, 2400, false, true, 20);
	chassis.turnTo(40, -2, 1000);
	INT.move(-127);
	leftWing.set_value(true);
	rightWing.set_value(true);
	driveMove(110);
	delay(800);
	chassis.setPose(41, -2, 90);
	driveMove(-80);
	delay(300);
	driveMove(0);
	leftWing.set_value(false);
	rightWing.set_value(false);

	// grab central safe triball
	chassis.turnTo(14, -17, 1000);
	INT.move(127);
	chassis.moveTo(14, -17, 235, 1500);
	chassis.turnTo(59, -45, 700);
	INT.move(0);
	chassis.moveTo(59, -45, 125, 2500, false, true, 35);

	// go back and knock out matchload
	rightWing.set_value(true);
	chassis.turnTo(45, -24, 1000);
	rightWing.set_value(false);
	chassis.turnTo(6, -55, 1000);
	INT.move(-127);
	delay(300);

	// grab 6 ball
	INT.move(127);
	chassis.moveTo(10, -54, 270, 2500, false, true, 25);

	// move back
	chassis.moveTo(40, -54, 270, 2500, false, false, 25);
	chassis.turnTo(61, -28, 1000);
	INT.move(-127);
	delay(300);
	INT.move(0);
	chassis.turnTo(61, -28, 1000, false, true);
	chassis.moveTo(61, -30, 180, 900, false, false, 55, .2);
	driveMove(-110);
	delay(600);
	driveMove(50);
	delay(400);
	driveMove(0);
}

void skills() {
	// initial setup and setting cata to global speed
	autoFireOn = true;
	// CR.move(globalCataSpeed);
	chassis.setPose(-49, -56, 225);

	// turn toward goal and fire for 40 seconds
	chassis.turnTo(46, -9, 1000, false, true);
	// pros::delay(35000); // however long it takes to fire all triballs

	// turn autofire on to stop cata from hitting bar
	autoFireOn = false;

	// turn and move backwards to other side
	// pushing triballs along with robot
	INT.move(-127);
	chassis.turnTo(-19, -61, 1000, false, true);
	chassis.moveTo(-19, -61, 285, 1000, false, false);
	chassis.moveTo(39, -56, 270, 2000, false, false);

	// backwards push corner triballs into goal
	chassis.turnTo(61, -43, 1000, false, true);
	chassis.moveTo(61, -43, 230, 1500, false, false);
	chassis.turnTo(64, 0, 1000, false, true);
	// chassis.moveTo(61, -32, 180, 2000, false, false, 0.6, 20);
	driveMove(-90);
	delay(1500);
	chassis.setPose(61, -32, 180);
	driveMove(20);
	delay(750);
	driveMove(0);

	// move out of the corner and towards the middle,
	// deploying wings on way to collect triballs
	chassis.turnTo(11, -36, 1000);
	chassis.moveTo(11, -36, -75, 1500);
	chassis.moveTo(11, -21, 0, 1500);

	// first front push 
	chassis.turnTo(48, -10, 1000);
	leftWing.set_value(true);
	rightWing.set_value(true);
	// chassis.moveTo(40, -5, 90, 1500, false, true, 20);
	driveMove(100);
	delay(1000);
	chassis.setPose(40, -11, 90);
	driveMove(-20);
	delay(1000);
	driveMove(0);
	delay(100);
	leftWing.set_value(false);
	rightWing.set_value(false);

	// back up and second push
	chassis.moveTo(11, -11, 90, 1500, false, false);
	chassis.turnTo(11, 29, 1000);
	chassis.moveTo(11, 34, 0, 1500);
	chassis.turnTo(45, 0, 1000);
	leftWing.set_value(true);
	rightWing.set_value(true);
	driveMove(100);
	delay(1200);
	leftWing.set_value(false);
	rightWing.set_value(false);
	chassis.setPose(40, 11, 90);
	driveMove(-60);
	delay(700);
	driveMove(0);
	chassis.turnTo(55, 50, 1000);
	chassis.moveTo(55, 50, 60, 3500, false, true, 30, .2);
	chassis.turnTo(58, 30, 800, false, true);
	chassis.moveTo(58, 30, 0, 1000, false, false, 30, .2);
	driveMove(50);
	delay(400);
	driveMove(0);

}

ASSET(skillspath1_txt);

void skills2() {
	chassis.follow(skillspath1_txt, 15000, 4);
	leftWing.set_value(true);
	rightWing.set_value(true);
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
	lcd::initialize();
	OPT1.set_led_pwm(30);
	OPT2.set_led_pwm(30);
	autoFireOn = true;
	//Task brainScreen(screenDisplay1)
	//Task controllerScreenTask(controllerScreen);
	Task autoPuncherTask(autoPuncher);
	Task autoLowerTask(autoLower);
	Task ledUpdaterTask(ledUpdater);
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
    if(selector::auton == -1){} // empty
    if(selector::auton == -2){fiveBallMidrush();} // 6 ball
    if(selector::auton == -3){sixBallMidrush();} // 5 ball
    if(selector::auton == 0){skills();} // skills
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
	if (competitionMode) {
		LV_IMG_DECLARE(BrainScreenIdle);
		lv_obj_t *img = lv_img_create(lv_scr_act(), NULL);
		lv_img_set_src(img, &BrainScreenIdle);
		lv_obj_align(img, NULL, LV_ALIGN_CENTER, 0, 0);
	}

	// reset old varaibles
	autoFireOn = false;
	blockerUp = false;
	autoLower = true;
	// initialize new variables
    float moveSpeed = .9;
    float turnSpeed = .5;
	bool leftWingOut = false;
	bool rightWingOut = false;
	bool cataMotorOn = false;
	// declare new variables to be used later for drivetrain code
	float move;
	float turn;
	float left;
	float right;

	// set drive motors to coast
	FL.set_brake_mode(E_MOTOR_BRAKE_COAST);
	FR.set_brake_mode(E_MOTOR_BRAKE_COAST);
	BL.set_brake_mode(E_MOTOR_BRAKE_COAST);
	BR.set_brake_mode(E_MOTOR_BRAKE_COAST);
	TL.set_brake_mode(E_MOTOR_BRAKE_COAST);
	TR.set_brake_mode(E_MOTOR_BRAKE_COAST);

	// set cata and intake motors to brake
	CR.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	INT.set_brake_mode(E_MOTOR_BRAKE_BRAKE);

	flow("0xFF0000", "FF0000");

	/*
	 * BUTTON INPUT SYSTEM
	*/

	while (true) {
		// ******************************************
		// CONTROLLER 1							   // 
		// ******************************************

		// toggle cata on/off with "A" button
		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_A)){
			if (autoFireOn) {
				autoFireOn = false;
				cataMotorOn = false;
			}
			cataMotorOn = !cataMotorOn
			if (cataMotorOn) {
				CR.move(globalCataSpeed);
			}
			else {
				CR.move(0);
			}
		}

		// toggle autoPuncher with "B" button
		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_B)) {
			autoFireOn = !autoFireOn
		}

		// toggle autoLower with "Y" button
		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_Y)) {
			autoFireOn = false;
			autoLower = !autoLower
		}

		// toggle blocker with "X" button
		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_X)) {
			blockerUp = !blockerUp
			blocker.set_value(blockerUp);
		}

		// for testing (not for match use); run selected autonomous with "DOWN" button
		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_DOWN)) {
			autonomous();
		}

		// decrease cata speed with "LEFT" button
		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_LEFT)) {
			if (globalCataSpeed > 70) {globalCataSpeed -= 5;}
		}

		// increase cata speed with "LEFT" button
		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_RIGHT)) {
			globalCataSpeed += 5;
			if (globalCataSpeed < 125) {globalCataSpeed += 5;}
		}

		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_UP)) {
			chassis.setPose(0,0,0);
			chassis.turnTo(100, 0, 2000);
		}

		// toggle left wing with "L2" button
		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_L2)){
			leftWingOut = !leftWingOut;
			leftWing.set_value(leftWingOut);
		}

		// toggle right wing with "L1" button
		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_L1)){
			rightWingOut = !rightWingOut;
			rightWing.set_value(rightWingOut);
		}

		// intake if holding R1, outtake if holding R2
		if (Controller1.get_digital(E_CONTROLLER_DIGITAL_R2)){
			INT.move(-127);
		}
		else if (Controller1.get_digital(E_CONTROLLER_DIGITAL_R1)){
			INT.move(127);
		} 
		else {
			INT.move(0);
		}

		// Double arcade drive controls - left joystick controls forward/backward movement, right joystick controls turning
		move = Controller1.get_analog(ANALOG_LEFT_Y);
		turn = Controller1.get_analog(ANALOG_RIGHT_X);
		left = (move * moveSpeed + turn * turnSpeed);
		right = (move * moveSpeed - turn * turnSpeed);
		// move drivetrain motors
		FL.move(left);
		TL.move(left);
		BL.move(left);
		FR.move(right);
		TR.move(right);
		BR.move(right);   

		// overheating warnings for all motors
		overheatWarning(FL);
        overheatWarning(TL);
        overheatWarning(BL);
        overheatWarning(FR);
        overheatWarning(TR);
        overheatWarning(BR);
        overheatWarning(CR);
        overheatWarning(INT);

		delay(10);
	}
}