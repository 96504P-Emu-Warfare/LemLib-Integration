#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
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
//Motor CL(18, E_MOTOR_GEARSET_18, 1);
Motor CR(20, E_MOTOR_GEARSET_18, 0);

MotorGroup leftMotors({BL, TL});
MotorGroup rightMotors({BR, TR});

ADIDigitalOut rightWing('A');
ADIDigitalOut leftWing('B');
ADIDigitalOut blocker('C');
ADILed driveLeds(4, 27);

bool blockerUp = false;
int globalCataSpeed = 90;
int cataDelay = 10; // in ms
bool autoLower;
bool competitionMode = false;
bool driveControlStarted = false;
bool autoFireOn;

Controller Controller1(CONTROLLER_MASTER);
Controller Controller2(CONTROLLER_PARTNER);

Optical OPT1(4);
Optical OPT2(9);

Imu Inr(8);

int triballsFired = 0;

bool flowOn = false;
bool flashOn = false;
bool sparkOn = false;
std::vector<uint32_t> colors;
u_int32_t tempColor;
int speed;

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
    65, // kP
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
        Controller1.set_text(2, 4, "OVERHEAT (" + std::to_string(motor.get_port()) + ", " + std::to_string(motor.get_temperature()) + ")");
    }
}

bool triballOnKicker() {
	//if detect triball green color on kicker
    double hue1 = OPT1.get_hue();
    if (OPT1.get_proximity() > 250) { //hue1 > 80 && hue1 < 95 && 
        return true;
    }
    return false;
}

void readyCata() {

	CR.move(127);

	while (OPT2.get_proximity() < 40) {
		delay(5);
	}
	CR.move(0);
}

void fireCata(int cataSpeed = 127) {

	CR.move(cataSpeed);

	while(triballOnKicker()) {

		delay(5);
	}

	CR.move(0);

}

void autoPuncher() {

	while (true) {

		while (autoFireOn) {
				
			readyCata();

			if (triballOnKicker()) {
				fireCata();
				triballsFired++;
			}

			delay(10);
		}

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

void driveMove(int power) {
	BL.move(power);
	BR.move(power);
	TL.move(power);
	TR.move(power);
	FL.move(power);
	FR.move(power);
}

// RGB CONTROL

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

std::vector<uint32_t> genGradient(uint32_t startColor, uint32_t endColor, size_t length) {
    std::vector<uint32_t> gradient;
    gradient.reserve(length);

    // Extract RGB components of startColor
    uint8_t startR = (startColor >> 16) & 0xFF;
    uint8_t startG = (startColor >> 8) & 0xFF;
    uint8_t startB = startColor & 0xFF;

    // Extract RGB components of endColor
    uint8_t endR = (endColor >> 16) & 0xFF;
    uint8_t endG = (endColor >> 8) & 0xFF;
    uint8_t endB = endColor & 0xFF;

    // Calculate the step size for each color component
    double stepR = static_cast<double>(endR - startR) / (length - 1);
    double stepG = static_cast<double>(endG - startG) / (length - 1);
    double stepB = static_cast<double>(endB - startB) / (length - 1);

    // Generate the gradient
    for (size_t i = 0; i < length; ++i) {
        uint8_t r = static_cast<uint8_t>(startR + (stepR * i));
        uint8_t g = static_cast<uint8_t>(startG + (stepG * i));
        uint8_t b = static_cast<uint8_t>(startB + (stepB * i));

        // Combine the RGB components into a single uint32_t color
        uint32_t color = (r << 16) | (g << 8) | b;
        gradient.push_back(color);
    }

    return gradient;
}

// update if more led strands are added
void set_pixel(u_int32_t color, int i) {
	driveLeds[i] = color;
}

void set_all(u_int32_t color) {
	driveLeds.set_all(color);
}

// tbh its probably just easier to make custom set pixel and set all functions to accomodate multiple string
// add public variables for turning on and off flow, and dif colors to flow

void flow(uint32_t color1, u_int32_t color2) {
	flowOn = true;
	flashOn = false;
	sparkOn = false;
	colors = genGradient(color1, color2, driveLeds.length());
}

void flash(uint32_t color, int flashSpeed){
	flashOn = true;
	flowOn = false;
	sparkOn = false;
	tempColor = color;
	speed = flashSpeed;
}

void spark(uint32_t color, int sparkSpeed) {
	sparkOn = true;
	flowOn = false;
	flashOn = false;
	tempColor = color;
	speed = sparkSpeed;
}

void LEDtask() {
	while (true) {

		if (autoFireOn && driveControlStarted)  {
			flowOn = false;
			flashOn = false;
			sparkOn = false;
			flash(0x00FF00, 8);
		}

		if (flowOn) {

			// loop through each pixel gets a color, update buffer, shift color matrix by 1, repeat
			for (int i = 0; i < driveLeds.length(); ++i) {
				set_pixel(colors[i], i);
			}

			set_pixel(colors[1], 0);
			set_pixel(colors[20], 1);
			
			// shift color vector
			std::rotate(colors.begin(), colors.begin()+1, colors.end());
		}

		if (flashOn) {
			set_all(tempColor);
			delay(speed*100);
			set_all(0x000000);
			delay(speed*100);
		}

		if (sparkOn) {
			sparkOn = false;
			for (int i = 0; i < driveLeds.length(); ++i) {
				set_pixel(tempColor, i);
				set_pixel(tempColor, i);
			}
		}

		delay(40);
	}
}

void turnOffLeds() {
	flowOn = false;
	flashOn = false;
}

void competitionTimerStuff() {
	delay(75000); // 30 seconds left
	flash(0xE9D502, 5);
	delay(15000); // 15 seconds left
	flash(0xD22730, 3);
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
	chassis.moveTo(-12, -55, 90, 3000);
}

void nearsideRush() {
	chassis.setPose(-45, -55, 48);

	// hit alliance triball toward goal with left wing
	rightWing.set_value(1);

	// grab central far triball
	INT.move(127);
	chassis.moveTo(-9, -9.5, 31, 300, false, true, 35);
	autoFireOn = true;
	rightWing.set_value(0);
	chassis.moveTo(-9, -9.5, 31, 2500, false, true, 35);
	delay(300);

	// back up wtih triball and move to match load bar
	driveMove(-70);
	delay(900);
	driveMove(0);
	chassis.moveTo(-50, -43, 30, 3000, false, false);

	// send match load and central triball down alley
	chassis.turnTo(-22, -81, 1000);
	rightWing.set_value(1);
	INT.move(-127);
	delay(300);
	chassis.turnTo(40, -30, 1000);
	rightWing.set_value(0);
	chassis.turnTo(-20, -56, 1000);
	chassis.moveTo(-12, -52, 90, 2500);
	driveMove(0);
	leftWing.set_value(1);
	delay(300);
	leftWing.set_value(0);
}

void fiveBallMidrush() {
	
}

void sixBallMidrush() {
	// release intake and set pose
	chassis.setPose(47, -53, 317);

	// hit alliance triball toward goal with right win
	rightWing.set_value(1);
	leftWing.set_value(1);

	// grab central far triball, turn and score both central and central far
	INT.move(127);
	chassis.moveTo(10, -6, 320, 200, false, true, 20);
	rightWing.set_value(0);
	leftWing.set_value(0);
	CR.move(0);
	autoFireOn = true;
	chassis.moveTo(10, -6, 320, 2400, false, true, 20);
	chassis.turnTo(40, -2, 1000);
	INT.move(-127);
	leftWing.set_value(1);
	rightWing.set_value(1);
	driveMove(110);
	delay(800);
	chassis.setPose(41, -2, 90);
	driveMove(-80);
	delay(300);
	driveMove(0);
	leftWing.set_value(0);
	rightWing.set_value(0);

	// grab central safe triball
	chassis.turnTo(14, -17, 1000);
	INT.move(127);
	chassis.moveTo(14, -17, 235, 1500);
	chassis.turnTo(59, -44, 700);
	INT.move(0);
	chassis.moveTo(59, -44, 125, 2500, false, true, 35);

	// go back and knock out matchload
	rightWing.set_value(1);
	INT.move(-127);
	chassis.turnTo(45, -24, 1000);
	rightWing.set_value(0);
	chassis.turnTo(6, -55, 1000);
	delay(300);

	// grab 6 ball
	INT.move(127);
	chassis.moveTo(12, -54, 270, 2500, false, true, 25);

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
	// initial setup
	chassis.setPose(-49, -56, 225);

	// turn toward goal and fire for x seconds, lower cata 
	chassis.turnTo(46, -9, 1000, false, true);
	CR.move(110);
	delay(500);
	autoFireOn = true;
	//pros::delay(31000); // however long it takes to fire all triballs

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
	driveMove(-90);
	delay(1500);
	chassis.setPose(61, -32, 180);
	driveMove(50);
	delay(1000);
	driveMove(-90);
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
	leftWing.set_value(1);
	rightWing.set_value(1);
	//chassis.moveTo(40, -5, 90, 1500, false, true, 20);
	driveMove(100);
	delay(1000);
	chassis.setPose(40, -11, 90);
	driveMove(-20);
	delay(1000);
	driveMove(0);
	delay(100);
	leftWing.set_value(0);
	rightWing.set_value(0);

	// back up and second push
	chassis.moveTo(11, -11, 90, 1500, false, false);
	chassis.turnTo(11, 29, 1000);
	chassis.moveTo(11, 34, 0, 1500);
	chassis.turnTo(45, 0, 1000);
	leftWing.set_value(1);
	rightWing.set_value(1);
	driveMove(100);
	delay(1200);
	leftWing.set_value(0);
	rightWing.set_value(0);
	chassis.setPose(40, 11, 90);
	driveMove(-60);
	delay(700);
	driveMove(0);

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
	//chassis.calibrate();
	lcd::initialize();
    selector::init();
	OPT1.set_led_pwm(30);
	OPT2.set_led_pwm(30);
	autoFireOn = true;
	Task autoPuncherTask(autoPuncher);
	Task ledUpdaterTask(ledUpdater);
	Task leds(LEDtask);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	flow(0xFFFFFF, 0xFF00FF);
}

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

	driveControlStarted = true;

	//Task brainScreen(screenDisplay1);
	Task controllerScreenTask(controllerScreen);

	if (competitionMode) {
		LV_IMG_DECLARE(BrainScreenIdle);
		lv_obj_t *img = lv_img_create(lv_scr_act(), NULL);
		lv_img_set_src(img, &BrainScreenIdle);
		lv_obj_align(img, NULL, LV_ALIGN_CENTER, 0, 0);
	}

	Task competitionTimerTask(competitionTimerStuff);

	autoFireOn = false;
	blockerUp = false;
	autoLower = true;

    // Variables
    float driveSpeed = .9;
    float turnSpeed = .5;
	bool leftWingOut = false;
	bool rightWingOut = false;
	bool cataMotorOn = false;

	FL.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	FR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	BL.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	BR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	TL.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	TR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

	CR.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	INT.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	flow(0xFFFFFF, 0xFF00FF);

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
			if (autoFireOn) {
				autoFireOn = false;
				cataMotorOn = false;
			}
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
			autoFireOn = false;
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
			if (globalCataSpeed <= 70) {globalCataSpeed = 70;}
		}

		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_RIGHT)) {
			globalCataSpeed += 5;
			if (globalCataSpeed >= 125) {globalCataSpeed = 125;}
		}

		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_UP)) {
			chassis.setPose(0,0,0);
			chassis.turnTo(100, 0, 2000);
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

		double left = (((drive * driveSpeed + turn * turnSpeed)));

		double right = (((drive * driveSpeed - turn * turnSpeed)));
		
		FL.move(left);
		TL.move(left);
		BL.move(left);
		FR.move(right);
		TR.move(right);
		BR.move(right);   
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