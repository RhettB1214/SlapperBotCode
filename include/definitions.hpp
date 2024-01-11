#ifndef DEFINITIONS
#define DEFINITIONS



//Controller Inputs
#define L1 pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_L1
#define L2 pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_L2
#define R1 pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_R1
#define R2 pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_R2
#define A pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_A
#define B pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_B
#define X pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_X
#define Y pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_Y
#define Up pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_UP
#define Down pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_DOWN
#define Right pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_RIGHT
#define Left pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_LEFT
#define LeftX pros::controller_analog_e_t::E_CONTROLLER_ANALOG_LEFT_X
#define LeftY pros::controller_analog_e_t::E_CONTROLLER_ANALOG_LEFT_Y
#define RightX pros::controller_analog_e_t::E_CONTROLLER_ANALOG_RIGHT_X
#define RightY pros::controller_analog_e_t::E_CONTROLLER_ANALOG_RIGHT_Y


//Gearbox Definitions
#define TORQUEBOX pros::motor_gearset_e_t::E_MOTOR_GEARSET_36
#define REGBOX pros::motor_gearset_e_t::E_MOTOR_GEARSET_18
#define SPEEDBOX pros::motor_gearset_e_t::E_MOTOR_GEARSET_06

//Unit Definitions
#define DEGREES pros::rotation_units_e_t::E_DEGREES
#define ROTATIONS pros::rotation_units_e_t::E_ROTATIONS

//Brake Mode Definitions
#define COAST pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_COAST
#define HOLD pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_HOLD


//Motor Port Definitions
#define INTAKE_PORT 2 //Defines a macro for the intake motor port and sets it to port 15
#define SLAP_PORT 20 //Defines a macro for the slapper motor port and sets it to port 20
#define LD1 1  //Defines a macro for the front left drive motor port and sets it to port 11
#define LD2 3 //Defines a macro for the bottom back left drive motor port and sets it to port 12
#define LD3 4 //Defines a macro for the top back left drive motor port and sets it to port 13
#define RD1 10 //Defines a macro for the front right drive motor port and sets it to port 20
#define RD2 9//Defines a macro for the bottom back right drive motor port and sets it to port 19 
#define RD3 8 //Defines a macro for the top back right drive motor port and sets it to port 18

//Misc Port Definitions
#define ODOM_ROT 21 //Defines a macro for the odometry rotation port and sets it to port 6
#define IMU_PORT 7 //Defines a macro for the imu sensor port and sets it to port 14


//Three-Wire Device Port Definitions
#define WING_ADIDO 'H' //Defines a macro for the pnuematic wing solenoid adi port and sets it to three wire port H
#define LIMIT_SWITCH 'A' //Defines a macro for the pnuematic arm solenoid adi port and sets it to three wire port G

//Short Hands For Chassis
#define poseX drive.getPose().x
#define poseY drive.getPose().y
#define poseTheta drive.getPose().theta

#endif /* DEFINITIONS */



