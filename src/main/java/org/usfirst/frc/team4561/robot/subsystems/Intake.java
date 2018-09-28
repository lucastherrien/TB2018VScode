package org.usfirst.frc.team4561.robot.subsystems;

import org.usfirst.frc.team4561.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
/**
 * This is the Intake subsystem
 * @author Karth, Lucas
 *
 */

public class Intake extends Subsystem {
//Two wheels: one turns left other turns right 	
	
	// Motors
	private WPI_TalonSRX intakeLeftMotor;
	private WPI_TalonSRX intakeRightMotor; 
	// Cube Detector
	public DigitalInput cubeDetector = new DigitalInput(0);
	//BobClaw
	public DoubleSolenoid bobClaw = new DoubleSolenoid(RobotMap.PCM, 6, 7);
	
	
	public Intake () {
		intakeLeftMotor = new WPI_TalonSRX (RobotMap.INTAKE_LEFT_MOTOR_PORT);
		intakeRightMotor = new WPI_TalonSRX (RobotMap.INTAKE_RIGHT_MOTOR_PORT);
		intakeLeftMotor.setInverted(false);
		intakeRightMotor.setInverted(true);
		intakeLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		intakeRightMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
	}
	
	public void bobClawOpen() {
		//Open the claw
		bobClaw.set(Value.kReverse);
	}
	
	public void bobClawClose() {
		//Close the claw
		bobClaw.set(Value.kForward);
	}
	
	public void bobStop() {
		bobClaw.set(Value.kOff);
	}
	
	public String bobState() {
		return bobClaw.get().toString();
	}
	
	//Intake speed
	public void leftIntake () {
		intakeLeftMotor.set(1);
	}
	public void rightIntake () {
		intakeRightMotor.set(1);
	}
	public void intakeIn(){
		intakeLeftMotor.set(0.8);
		intakeRightMotor.set(0.8);
	}
	public void intakeOutHalf(){
		intakeLeftMotor.set(-0.35);
		intakeRightMotor.set(-0.35);
	}
	public void intakeOutWeak() {
		intakeLeftMotor.set(-0.25);
		intakeRightMotor.set(-0.25);
	}
	public void stop(){
		intakeLeftMotor.set(0);
		intakeRightMotor.set(0);
	}
	public void leftIntakeStop () {
		intakeLeftMotor.set(0);
	}
	public void intakeForever() {
		intakeLeftMotor.set(3.5/13.5);
		intakeRightMotor.set(3.5/13.5);
	}
	public void rightIntakeStop () {
		intakeRightMotor.set(0);
	}
	
	public void release () {
		intakeLeftMotor.set(-0.85);
		intakeRightMotor.set(-0.85);
	}
	
	public void set(double left, double right){
		intakeLeftMotor.set(left);
		intakeRightMotor.set(right);
	}

	//These four methods are mainly used in the Intake debug
	public double getIntakeLeftPosition() {
    	return intakeLeftMotor.getSelectedSensorPosition(0);
    }
	
	public double getIntakeRightPosition() {
    	return intakeRightMotor.getSelectedSensorPosition(0);
    }
	
	public double getIntakeLeftVelocity() {
		return intakeLeftMotor.getSelectedSensorVelocity(0);
	}
	
	public double getIntakeRightVelocity() {
		return intakeRightMotor.getSelectedSensorVelocity(0);
	}
	
	
	public void setLeftIntake(double power) {
	     intakeLeftMotor.set(ControlMode.PercentOutput, power);
	 }
	    
    public void setRightIntake(double power) {
	     intakeRightMotor.set(ControlMode.PercentOutput, power);
	   
	   }


	public boolean detectorState() {
		return cubeDetector.get();
	}
	@Override
	protected void initDefaultCommand() {
		//setDefaultCommand(new IntakeDrive());

	}

}
