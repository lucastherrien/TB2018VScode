package org.usfirst.frc.team4561.robot.subsystems;

import org.usfirst.frc.team4561.robot.RobotMap;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;



import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * This is the Elavator PID subsystem
 * @author Snehil
 */

//TODO: Remove placeholder values
public class ElevatorPID extends Subsystem {
	
	private ControlMode follower = com.ctre.phoenix.motorcontrol.ControlMode.Follower;

	private WPI_TalonSRX motorOne = new WPI_TalonSRX(RobotMap.ELEVATOR_MOTOR_1_PORT);
	private WPI_TalonSRX motorTwo = new WPI_TalonSRX(RobotMap.ELEVATOR_MOTOR_2_PORT);
	
	private int goal = 31;
	
	public ElevatorPID() {
		//motorOne.set(RobotMap.ELEVATOR_MOTOR_1_PORT);
		motorOne.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
		motorTwo.set(follower, RobotMap.ELEVATOR_MOTOR_1_PORT);
		
		motorTwo.setInverted(false);
		motorOne.setInverted(true);
		motorOne.setSensorPhase(true);
		
		motorOne.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
		motorOne.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
		motorOne.configSetParameter(ParamEnum.eClearPositionOnLimitR, 0, 0, 0, 0);
		motorOne.overrideLimitSwitchesEnable(true);
		motorOne.setSelectedSensorPosition(motorOne.getSensorCollection().getAnalogInRaw(), 0, 0);
		
		motorOne.configForwardSoftLimitThreshold(610, 0);
		motorOne.configForwardSoftLimitEnable(true, 0);
		
		motorOne.configPeakCurrentLimit(20, 0);
		motorOne.enableCurrentLimit(false);
		
		motorOne.configPeakOutputForward(1, 0);
		motorOne.configPeakOutputReverse(-1, 0);
		motorTwo.configPeakOutputForward(1, 0);
		motorTwo.configPeakOutputReverse(-1, 0);
		
		motorOne.config_kP(0, 8, 0);
		motorOne.config_kI(0, 0, 0);
		motorOne.config_kD(0, 1023, 0);
		
		motorOne.enableCurrentLimit(false);
		
		motorOne.configOpenloopRamp(0.5, 0);
		motorTwo.configOpenloopRamp(0.5, 0);
	}
	
	public void resetGoal() {
		goal = (int) getElevatorPos();
		if (RobotMap.ELEVATOR_PID) motorOne.set(ControlMode.Position, goal);
	}
	public void resetBetter() {
		goal = (int) getElevatorPos();
	}
	public void limit() {
		motorOne.configPeakCurrentLimit(40, 0);
		motorOne.enableCurrentLimit(true);
	}
	public void unlimit() {
		motorOne.enableCurrentLimit(false);
	}
	public void set(double speed){
		motorOne.set(ControlMode.PercentOutput, speed);
	}
	public double getThrottle() {
		return motorOne.getMotorOutputPercent();
	}
	public void setToGoal(){
		if (Math.abs(goal - getElevatorPos()) > 10){
			motorOne.set(ControlMode.Position, goal);
		}
		else{
			motorOne.set(ControlMode.Position, goal);
		}
	}
	public void setGoalRelative(int gol){
		goal = gol;
	}
	public void stop(){
		motorOne.set(ControlMode.PercentOutput, 0);
	}
	//Elevator position with Arm touching ground
	public void GroundPosition() {
		goal = 31;
		if (RobotMap.ELEVATOR_PID) setToGoal();
	}
	
	//Elevator position with Arm at switch height
	public void SwitchPosition() {
		goal = 153;
		if (RobotMap.ELEVATOR_PID) setToGoal();
	}
	public void setToInches(double inches){
		goal = (int) (inches/16.5);
		if (RobotMap.ELEVATOR_PID) setToGoal();
	}
	//Elevator position with Arm at scale height
	public void ScalePositionMidFlat() {
		goal = 551;
		if (RobotMap.ELEVATOR_PID) setToGoal();
	}
	public void ScalePositionLowFlat(){
		goal = 457;
		if (RobotMap.ELEVATOR_PID) setToGoal();
	}
	public void ScalePositionHighFlat(){
		goal = 610;
		if (RobotMap.ELEVATOR_PID) setToGoal();
	}
	//Elevator position with Arm at scale height and arm flat
	public void ScalePositionMidArmAngled() {
		goal = 551;
		if (RobotMap.ELEVATOR_PID) setToGoal();
	}
	public void ScalePositionAuto() {
		goal = 551+83;
		if (RobotMap.ELEVATOR_PID) setToGoal();
	}
	public void ScalePositionLowArmAngled() {
		goal = 200;
		if (RobotMap.ELEVATOR_PID) setToGoal();
	}
	public void ScalePositionHighArmAngled() {
		goal = 610;
		if (RobotMap.ELEVATOR_PID) setToGoal();
	}
	public void resetFlow() {
		goal = motorOne.getSelectedSensorPosition(0) + (motorOne.getSelectedSensorVelocity(0));
	}
    public void setPowerOne(double power) {
           motorOne.set(ControlMode.PercentOutput, power);
           
       }
       
    public void setPowerTwo(double power) {
           motorTwo.set(ControlMode.PercentOutput, power);
           
       }

	public double getElevatorPos(){
		return motorOne.getSelectedSensorPosition(0);
	}
	
	public double getElevatorSpeed(){
		return motorOne.getSelectedSensorVelocity(0);
	}
	
	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		//setDefaultCommand(new ElevatorDrive());

	}

	public double motorOneVoltage(){
		return motorOne.getMotorOutputVoltage();
	}
	public double motorTwoVoltage(){
		return motorTwo.getMotorOutputVoltage();
	}
	public boolean limitSwitch(){
		return motorOne.getSensorCollection().isRevLimitSwitchClosed();
	}
	public double getGoal(){
		return goal;
	}
	public boolean nearGoal(){
		return Math.abs(motorOne.getClosedLoopError(0))<16;
	}
	public double getCurrentOne(){
		return motorOne.getOutputCurrent();
	}
	public double getCurrentTwo(){
		return motorTwo.getOutputCurrent();
	}
	public int getPotVolt(){
		return motorOne.getSensorCollection().getAnalogIn();
	}
}
