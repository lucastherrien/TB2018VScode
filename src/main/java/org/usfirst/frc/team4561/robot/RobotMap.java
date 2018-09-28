/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4561.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */

//TODO: Remove placeholder values

public class RobotMap {
	
	//Joystick Ports
	public static final int LEFT_JOYSTICK_PORT = 0;
	public static final int RIGHT_JOYSTICK_PORT = 1;
	public static final int CONTROLLER_PORT = 2;
	
	//Buttons
	public static final int RELEASE_BUTTON = 4;
	public static final int INTAKE_BUTTON = 1;
	public static final int OUTTAKE_FULL_BUTTON = 6;
	
	public static final int INTAKE_POSITION_BUTTON = 2;
	public static final int RELEASE_POSITION_BUTTON = 2;
	
	public static final int CONTROLLER_INTAKE = 2;
	public static final int ARM_BOTTOM_POS = 1;
	public static final int ARM_MIDDLE_POS = 3;
	public static final int ARM_TOP_POS= 4;
	
	public static final int CONTROLLER_LEFT_INTAKE = 5;
	public static final int CONTROLLER_RIGHT_INTAKE = 6;
	public static final int INTAKE_INFINITE_BUTTON = 5;
	
	public static final int ARM_UP_POV = 0;
	public static final int ARM_MIDDLE_POV1 = 90;
	public static final int ARM_MIDDLE_POV2 = 270;
	public static final int ARM_DOWN_POV = 180;
	
	public static final int TRANSMISSION_SPEED_BUTTON = 5;
	public static final int TRANSMISSION_TORQUE_BUTTON = 3;
	
	public static final int OUTTAKE_WEAK_BUTTON = 3;
	
	public static final double RIGHT_JOYSTICK_DEAD_ZONE = 0.25;
	public static final double LEFT_JOYSTICK_DEAD_ZONE = 0.25;
	public static final double RIGHT_JOYSTICK_REDUCTION = 0.25;
	public static final double LEFT_JOYSTICK_REDUCTION = 0.25;
	
	
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;
	
	//Left Motor Ports
	public static final int FRONT_LEFT_MOTOR_PORT = 7; //7
	public static final int MID_LEFT_MOTOR_PORT = 11; //11
	public static final int BACK_LEFT_MOTOR_PORT = 12;
	
	//Right Motor Ports
	public static final int FRONT_RIGHT_MOTOR_PORT = 8;
	public static final int MID_RIGHT_MOTOR_PORT = 9;
	public static final int BACK_RIGHT_MOTOR_PORT = 10;
		
	
	//Drive Mode 
	public static final int DRIVE_MODE = 1; // 1 is arcade drive, 0 is tank drive
	
	// Elevator Ports (zeros for placeholders)
	public static final int ELEVATOR_MOTOR_1_PORT = 1;
	public static final int ELEVATOR_MOTOR_2_PORT = 2;	
	
	// Arm ports (zeros for placeholders)
	public static final int ARM_MOTOR_1_PORT = 6;

	//Intake ports
	public static final int INTAKE_LEFT_MOTOR_PORT = 3;
	public static final int INTAKE_RIGHT_MOTOR_PORT = 4;
	
	//Transmission Ports
	public static final int TRANSMISSION_SOLENOID_PORT = 0;
	public static final int TRANSMISSION_SOLENOID_PORT_TWO = 1;
	public static final int TRANSMISSION_SOLENOID_TWO_PORT = 2;
	public static final int TRANSMISSION_SOLENOID_TWO_PORT_TWO = 3;
	public static final int PCM = 13;

	//Toggle PID Buttons
	public static final int TOGGLE_ARM_BUTTON = 9;
	public static final int TOGGLE_ELEVATOR_BUTTON = 10;
	public static final int TOGGLE_DRIVETRAIN_BUTTON_ONE = 11;
	public static final int TOGGLE_DRIVETRAIN_BUTTON_TWO = 9;
	public static final int TOGGLE_DRIVETRAIN_BUTTON_THREE = 7;
	public static final int TOGGLE_PID_BUTTON_ONE = 8;
	public static final int TOGGLE_PID_BUTTON_TWO = 7;
	
	public static final int BOB_CLAW_BUTTON = 1;
	
	// Debug variables
	public static final boolean MASTER_DEBUG = true;
	@SuppressWarnings("unused")
	public static final boolean DRIVETRAIN_DEBUG = true || MASTER_DEBUG;
	public static final boolean ELEVATOR_DEBUG = false || MASTER_DEBUG;
	public static final boolean TRANSMISSION_DEBUG = false || MASTER_DEBUG;
	public static final boolean ARM_DEBUG = false || MASTER_DEBUG;
	
	public static final double DRIVETRAIN_CIRCUMFERENCE = 88;
	public static final double CONTROLLER_DEADZONE = 0.1;
	
	//PID variables
	public static boolean MASTER_PID = false;
	public static boolean DRIVETRAIN_PID = false || MASTER_PID;
	@SuppressWarnings("unused")
	public static boolean ELEVATOR_PID = true || MASTER_PID;
	@SuppressWarnings("unused")
	public static boolean ARM_PID = true || MASTER_PID;
	
	// Automode constraints
	public static final double TIME_STEP = 0.05; // sec
	public static final double WHEELBASE_WIDTH_SWITCH = 2.55;//2.25; // 2.8;// 3.3;//3.1; //7; // ft: 1.865 for Delta (fudged fo 2.95), 1.375 for Kongo, 1.865 for Janderson
	public static final double WHEELBASE_WIDTH_SCALE = 2.55;//2.7;
	public static final double MAX_VELOCITY = 2; //ft/sec: 15.9 for Delta (decreased to 13.9) in speed gear
	public static final double MAX_ACCELERATION = 1; // ft/s/s: 6 for Delta
	public static final double MAX_JERK = 60.0; // ft/s/s/s
	
	public static final boolean LEFT_SIDE_INVERTED = false; // Delta: false, Kongo: false, Janderson: false
	public static final boolean RIGHT_SIDE_INVERTED = true; // Delta: true, Kongo: true, Janderson: true
	public static final boolean LEFT_SIDE_SENSOR_PHASE_REVERSED = false;
	public static final boolean RIGHT_SIDE_SENSOR_PHASE_REVERSED = false;
	
	// Used for MotionProfileOnboardRunner
	public static final double WHEEL_DIAMETER = 5; //inches: 5 for Delta, 6 for Kongo, 3.5 for Janderson
	public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
	public static final int UNITS_PER_REVOLUTION = (int)((RobotMap.UNITS_PER_10_FEET / 10.0 * (WHEEL_CIRCUMFERENCE / 12.0)) * 1.0); //encoder ticks: 7659 for Delta, 8192 for Kongo, 3700 for Janderson
	public static final double UNITS_PER_10_ROBOT_REVOLUTIONS = 410000; // 410000 for Delta, 274700 for Janderson (currently unused), using WHEELBASE_WIDTH instead
	public static final double UNITS_PER_10_FEET = 59500; // 59500 for Delta, 40500 for Janderson
	public static final double MAX_UNITS_PER_100MS = 2500 * 1.505; // 9900 for Delta, 4011 for Janderson, 6450 for Kongo
	public static final double MAX_UNITS_PER_SECOND = MAX_UNITS_PER_100MS * 10;
	public static final double MAX_REVOLUTIONS_PER_SECOND = MAX_UNITS_PER_SECOND / UNITS_PER_REVOLUTION;
	public static final double MAX_INCHES_PER_SECOND = MAX_REVOLUTIONS_PER_SECOND * WHEEL_DIAMETER * Math.PI;
	public static final double MAX_FEET_PER_SECOND = MAX_INCHES_PER_SECOND / 12;
	
	public static final double ONBOARD_ENCODER_MULTIPLIER = 1;

	
	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
}
