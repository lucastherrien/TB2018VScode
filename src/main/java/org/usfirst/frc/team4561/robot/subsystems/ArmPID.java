package org.usfirst.frc.team4561.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import org.usfirst.frc.team4561.robot.RobotMap;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

/** 
 * This is the Arm PID subsystem
 * @author Max
 *
 */
public class ArmPID extends Subsystem {
	private WPI_TalonSRX motorOne;
	private int goal = -120;
	private int velGoal = 0;
	private int speed = 1000;
	//private WPI_TalonSRX motorTwo;
	public ArmPID() {
		motorOne = new WPI_TalonSRX(RobotMap.ARM_MOTOR_1_PORT);
		motorOne.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		motorOne.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
		motorOne.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
		motorOne.configSetParameter(ParamEnum.eClearPositionOnLimitR, 0, 0, 0, 0);
		motorOne.configSetParameter(ParamEnum.eClearPositionOnLimitF, 1, 0, 0, 0);
		motorOne.setSensorPhase(true);
		motorOne.setInverted(true);
		//motorTwo = new WPI_TalonSRX(RobotMap.ARM_MOTOR_2_PORT);
		//motorTwo.set(followerRobotMap.ARM_MOTOR_1_PORT);
		motorOne.config_kP(0, 4, 0);
		motorOne.config_kI(0, 0.0055, 0);
		motorOne.config_kD(0, 1023, 0);
		motorOne.config_kP(1, 4, 0);
		motorOne.config_kF(1, 3.41, 0);
		motorOne.config_IntegralZone(0, 120, 0);
		motorOne.configPeakOutputForward(1, 0);
		motorOne.configPeakOutputReverse(-1, 0);
		motorOne.configNominalOutputForward(0, 0);
		motorOne.configNominalOutputReverse(0, 0);
		motorOne.configAllowableClosedloopError(0, 0, 0);
		motorOne.enableCurrentLimit(false);
		motorOne.configOpenloopRamp(0, 0);
		motorOne.selectProfileSlot(0, 0);
		motorOne.configClosedloopRamp(0.2, 1);
	}
	// Arm on ground to intake block
	public double getThrottle() {
		return motorOne.getMotorOutputPercent();
	}
	public void clear() {
		motorOne.setIntegralAccumulator(0, 0, 0);
	}
	public void IntakePosition() {
		goal = -1560;
		if (RobotMap.ARM_PID) setToGoal();
		if (RobotMap.ARM_DEBUG) {
			System.out.println("[Subsystem] ArmPID: Down to Intake Position");
			}
		}
	public double getVoltage(){
		return motorOne.getMotorOutputVoltage();
	}
	public void limit() {
		motorOne.configPeakCurrentLimit(40, 0);
		motorOne.enableCurrentLimit(true);
		}
	public void unlimit() {
		motorOne.enableCurrentLimit(false);
	}
	//Arm in straight release position
	public void ReleasePosition() {
		goal = -1370;
		if (RobotMap.ARM_PID) setToGoal();
		if (RobotMap.ARM_DEBUG) {
			System.out.println("[Subsystem] ArmPID: Up to Release Position");
		}
	}
	public void AnglePosition() {
		goal = -919;
		if (RobotMap.ARM_PID) {
			setToGoal();
		}
	}
	public void DiagonalPosition(){
		goal = -334;
		if (RobotMap.ARM_PID) setToGoal();
		if (RobotMap.ARM_DEBUG){
			System.out.println("Subsystem ArmPID: Diagonal Position");
		}
	}
	public void UpPostition(){
		goal = -167;
		if (RobotMap.ARM_PID) setToGoal();
		if (RobotMap.ARM_DEBUG){
			System.out.println("[Subsystem] ArmPID: Up to Up Position");
		}
	}
	/**
     * Gets the encoder position of the arm.
     */
    public void setToGoal(){
    	motorOne.selectProfileSlot(0, 0);
    	motorOne.configClosedloopRamp(0.2, 0);
    	clear();
    	motorOne.set(ControlMode.Position, goal);
    }
    public void setGoal(int gol){
    	goal = gol;
    }
    public double getEncoderPosition() {
    	return motorOne.getSelectedSensorPosition(0);
    }
    public void set(double speed){
//    	motorOne.selectProfileSlot(1, 0);
//    	motorOne.configClosedloopRamp(0, 0);
    	clear();
    	velGoal = (int) speed*300;
    	motorOne.set(ControlMode.PercentOutput, speed);
    }
    public int getVGoal() {
    	return velGoal;
    }
    public boolean isVelocity() {
    	return motorOne.getControlMode() == ControlMode.PercentOutput;
    }
    public void resetFlow() {
    	goal = motorOne.getSelectedSensorPosition(0)+ (int) (1.25*motorOne.getSelectedSensorVelocity(0));
    }
    /**
     * Gets the encoder velocity of the arm.
     */
    public void resetGoal(){
    	goal = motorOne.getSelectedSensorPosition(0);
    	if (RobotMap.ARM_PID) motorOne.set(ControlMode.Position, goal);
    }
    public void resetBetter() {
    	goal = motorOne.getSelectedSensorPosition(0);
    }
    public double getEncoderVelocity() {
    	return motorOne.getSelectedSensorVelocity(0);
    }
    public void stop(){
    	motorOne.set(ControlMode.PercentOutput, 0);
    }
	@Override
	protected void initDefaultCommand() {
		//setDefaultCommand(new ArmDrive());

	}
	
	public void setEncoderPos(int pos){
		motorOne.setSelectedSensorPosition(pos, 0, 0);
	}
	public boolean nearGoal(){
		boolean yes = Math.abs(getEncoderPosition()-goal)<15;
		if (yes) System.out.println("Yes");
		return yes;
	}
	public boolean getFwdSwitch(){
		return motorOne.getSensorCollection().isFwdLimitSwitchClosed();
	}
	public boolean getRevSwitch(){
		return motorOne.getSensorCollection().isRevLimitSwitchClosed();
	}
	
	public int getGoal(){
		return goal;
	}
	public double getCurrent(){
		return motorOne.getOutputCurrent();
	}
}
