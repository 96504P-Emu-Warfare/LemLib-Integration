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

Imu Inr(16);

lemlib::Drivetrain_t drivetrain {
	&leftMotors,
	&rightMotors,
	10, // distance between front wheels
	3.25, // wheel diameter
	360 // wheel rpm (gearing * motor rpm)
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
	nullptr, //vertical tracking wheel 1
	nullptr, //vertical tracking wheel 2
	nullptr,
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
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
    if (motor.get_temperature() >= 74) {
        Controller1.set_text(2, 8, "OVERHEAT");
        Controller1.rumble("......");
        delay(400);

    }
}

void driverControl() {

	delay(50);

	Controller1.set_text(1,1,"Drivecontrol started");

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

	/**
	 * BUTTON INPUT SYSTEM
	 */
	

	while (true)
	{
		//void controllerScreenSetupEMU();

		// ******************************************
		// CONTROLLER 1							   //
		// ******************************************

/**
		if (Controller1.get_digital_new_press(E_CONTROLLER_DIGITAL_A)){
			if (cataMotorOn == false) {
				CL.move_velocity(70);
				cataMotorOn = true;
			}
			else {
				CL.move_velocity(0);
				cataMotorOn = false;
			}
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
		*/

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

		overheatWarning(FL);
        overheatWarning(TL);
        overheatWarning(BL);
        overheatWarning(FR);
        overheatWarning(TR);
        overheatWarning(BR);
        overheatWarning(CL);
        overheatWarning(CR);
        overheatWarning(INT);
        

		pros::delay(20);

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
	lcd::set_text(1, "Hello PROS User!");
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
	driverControl();
}

