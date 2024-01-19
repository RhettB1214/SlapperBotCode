#include "main.h"
#include "lemlib/api.hpp"
#include "autoSelect/selection.h"
#include "gif-pros/gifclass.hpp"
#include "definitions.hpp"




//ASSET Defintions

	/*Red Far*/
	ASSET(RedFar1_txt);

	/*Red Close*/
	ASSET(RedClose1_txt);
	ASSET(RedClose2_txt);
	ASSET(RedClose3_txt);
	ASSET(RedClose4_txt);

	/*Red Safe AWP Far*/

	/*Blue Far*/


	/*Blue Close*/

	
	/*Blue Safe AWP Far*/



	/*Skills*/


/*End of ASSET Defintions*/

/*Variable Definitions*/

    /*Controller Variables*/
    bool lastKnownButtonR1State;
	bool lastKnownButtonBState;
	bool lastKnownButtonUpState;
	bool wingToggle = false;
	bool actuatateWings = false;

	/*Cata Variables*/
	bool slapperFireToggle = false;


/*End of Variable Definitions*/


/*Device Initilization*/

	/*Drivetrain Initilizations*/

	pros::Motor lD1(LD1, SPEEDBOX, true);
	pros::Motor lD2(LD2, SPEEDBOX, true);
	pros::Motor lD3(LD3, SPEEDBOX, true);
	pros::Motor rD1(RD1, SPEEDBOX, false);
	pros::Motor rD2(RD2, SPEEDBOX, false);
	pros::Motor rD3(RD3, SPEEDBOX, false);

	pros::MotorGroup lDrive({lD1, lD2, lD3});
	pros::MotorGroup rDrive({rD1, rD2, rD3});

	pros::Imu imu(IMU_PORT);

	pros::Rotation odomRot(ODOM_ROT, false);

	lemlib::TrackingWheel odomWheel(&odomRot, 2.75, 0, 1);

	/*End of Drivetrain Initializations*/


	/*Non-DT Initializations*/

	pros::Motor slapperMotor(SLAP_PORT, TORQUEBOX, true);
	pros::Motor intakeMotor(INTAKE_PORT, SPEEDBOX, false);

	pros::ADIDigitalOut wingPnuem(WING_ADIDO);



	/*End of Non-DT Initializations*/

	/*Controller Initialization*/

	pros::Controller master(pros::E_CONTROLLER_MASTER);

	/*End of Controller Initilization*/


/*End of Device Initilization*/


/*LemLib Chassis Initializations*/

	/*LemLib Drivetrain Initilization*/
	lemlib::Drivetrain drivetrain
	{
		&lDrive, /*Pointer to the left drive channel*/
		&rDrive, /*Pointer to the right drive channel*/
		10.5, /*Track Width*/
		3.25, /*Wheel Diameter*/
		450, /*Wheel RPM*/
		8 /*Chase Power*/
	};
	/*End of LemLib Drivetrain Initilization*/


	/*LemLib Odometry Initilization*/
	lemlib::OdomSensors odomSensors
	{
		&odomWheel, /*Center Wheel*/
		nullptr, /*No Tracking Wheel*/
		nullptr, /*No Tracking Wheel*/
		nullptr, /*No Tracking Wheel*/
		&imu /*Inertial Sensor*/
	};
	/*End of LemLib Odometery Sensors Initilization*/


	/*Lateral (Forwards/Backwards) PID Initilization*/
	lemlib::ControllerSettings lateralController
	(
		16,  //16, // kP
        3, // kI
		80, //72, // kD
        4, // Windup Range
		1, // smallErrorRange
		100, // smallErrorTimeout
		3, // largeErrorRange
		500, // largeErrorTimeout
		10 // Slew Rate
    );
	/*End of Lateral (Forwards/Backwards) PID Initilization*/


	/*Angular (Turning) PID Initilization*/
	lemlib::ControllerSettings angularController(
		7,  //10 // kP
        0, // kI
		60, //60 // kD
        0, // Windup Range
		1, // smallErrorRange
		100, // smallErrorTimeout
		3, // largeErrorRange
		500, // largeErrorTimeout
		10 // Slew Rate
    );
	/*End of Angular (Turning) PID Initilization*/


	/*LemLib Chassis Initilization*/
	lemlib::Chassis drive(drivetrain, lateralController, angularController, odomSensors);
	/*End of LemLib Chassis Initilization*/


/*End of LemLib Chassis Initializations*/


void screen() {
    // loop forever
    while (true) {
        lemlib::Pose pose = drive.getPose(); // get the current position of the robot
        pros::lcd::print(0, "x: %f", pose.x); // print the x position
        pros::lcd::print(1, "y: %f", pose.y); // print the y position
        pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
        pros::delay(10);
    }
}


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() 
{
	drive.calibrate();
	intakeMotor.set_brake_mode(HOLD);
	slapperMotor.set_brake_mode(COAST);
	selector::init();
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
void autonomous() 
{
	
	lDrive.set_brake_modes(HOLD);
	rDrive.set_brake_modes(HOLD);
	//drive.setPose({0,0,0});
	//drive.moveToPose(0, 24, 0, 3000);

		    /*drive.setPose({15.5, -60, 270});
			intakeMotor.move(127);
			drive.moveToPose(8, -60, 270, 500);
			drive.waitUntilDone();
			pros::delay(150);
			drive.moveToPose(15.5, -60, 270, 500,{.forwards = false});
			drive.waitUntilDone();
			pros::delay(100);
			drive.follow(RedFar1_txt, 15, 2500, false);*/


	switch(selector::auton)
	{
		//Red Far
		case 1:
		{
			drive.setPose({15.5, -60, 270});
			intakeMotor.move(127);
			drive.moveToPose(8, -60, 270, 500);
			drive.waitUntilDone();
			pros::delay(150);
			drive.follow(RedFar1_txt, 15, 2500, false);
		}

		//Red Close
		case 2:
		{
			drive.setPose({-32, -55, 270});
			drive.follow(RedClose1_txt, 15, 2000);
			drive.waitUntil(26);
			intakeMotor.move(-127);
			drive.waitUntilDone();
			intakeMotor.move(0);
			drive.follow(RedClose2_txt, 15, 1500, false);
			drive.waitUntil(4);
			wingPnuem.set_value(1);
			drive.waitUntilDone();
			drive.follow(RedClose3_txt, 15, 2000);
			drive.waitUntilDone();
			wingPnuem.set_value(0);
			pros::delay(500);
			drive.follow(RedClose4_txt, 15, 1500, false);

		}

		//Red Safe AWP Far
		case 3:
		{

		}

		//Blue Far
		case -1:
		{

		}

		//Blue Close
		case -2:
		{

		}

		//Blue Safe AWP Far
		case -3:
		{

		}

		//Skills
		case 0:
		{
			
		}
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
void opcontrol() 
{
	lDrive.set_brake_modes(COAST);
	rDrive.set_brake_modes(COAST);

	while(true)
	{
		drive.tank(masterLeftY, masterRightY, 5);


		if (masterL1)
		{
			intakeMotor.move(-127);
		}
		else if (masterL2) 
		{
			intakeMotor.move(127);
		}
		else
		{
			intakeMotor.move(0);
		}


		if (masterR1 != lastKnownButtonR1State)
		{
			lastKnownButtonR1State = masterR1;
			if (masterR1)
			{
				slapperFireToggle = !slapperFireToggle;
			}
		}

		if (slapperFireToggle)
		{
			slapperMotor.move(115);
		}
		else
		{
			slapperMotor.move(0);
		}

		if (masterR2)
		{
			wingPnuem.set_value(1);
		}
		else if (!wingToggle)
		{
			wingPnuem.set_value(0);
		}

		if(masterUp != lastKnownButtonUpState)
		{
			lastKnownButtonUpState = masterUp;
			if(masterUp)
			{
				wingToggle = !wingToggle;
				actuatateWings = !actuatateWings;
			}
		}

		if (wingToggle && actuatateWings)
		{
			wingPnuem.set_value(1);
		}





	}


	
}
