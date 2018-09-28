package org.usfirst.frc.team4561.robot.subsystems;

import org.usfirst.frc.team4561.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Non-PID arm subsystem.
 * @author Ben
 */
public class Arm extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	private WPI_TalonSRX motorOne;
	public Arm() {
		motorOne = new WPI_TalonSRX(RobotMap.ARM_MOTOR_1_PORT);
		motorOne.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		motorOne.setSensorPhase(true);
		//motorTwo = new WPI_TalonSRX(RobotMap.ARM_MOTOR_2_PORT);
		//motorTwo.set(follower, RobotMap.ARM_MOTOR_1_PORT);
	}
	
	public void down() {
		// down (half speed)
		motorOne.set(-0.1);
		if (RobotMap.ARM_DEBUG) {
			System.out.println("[Subsystem] Non-PID Arm: Going down");
		}
	}
	
	public void up() {
		// up (half speed)
		motorOne.set(0.1);
		if (RobotMap.ARM_DEBUG) {
			System.out.println("[Subsystem] Non-PID Arm: Going up");
		}
	}
	
	public void stop() {
		// stop
		motorOne.set(0);
		if (RobotMap.ARM_DEBUG) {
			System.out.println("[Subsystem] Non-PID Arm: Stopped");
		}
	}
	
	/**
     * Gets the encoder position of the arm.
     */
    
    public double getEncoderPosition() {
    	return motorOne.getSelectedSensorPosition(0);
    }
    
    public void resetEncoder(){
    	motorOne.setSelectedSensorPosition(0, 0, 0);
    }
    
    /**
     * Gets the encoder velocity of the arm.
     */
    
    public double getEncoderVelocity() {
    	return motorOne.getSelectedSensorVelocity(0);
    }
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

//41204F4B0D0A