package org.usfirst.frc.team4561.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team4561.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * This is the non-PID elevator subsystem file.
 * @author Ben
 */
public class Elevator extends Subsystem {

    private WPI_TalonSRX motorOne;
    private WPI_TalonSRX motorTwo;

    public Elevator() {
    	motorOne = new WPI_TalonSRX(RobotMap.ELEVATOR_MOTOR_1_PORT);
    	motorOne.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
    	motorTwo = new WPI_TalonSRX(RobotMap.ELEVATOR_MOTOR_2_PORT);
    	motorTwo.set(ControlMode.Follower, RobotMap.ELEVATOR_MOTOR_1_PORT);
    }
    
    public void climbUp() {
    	// full forward
    	motorOne.set(1);
    	if (RobotMap.ELEVATOR_DEBUG) {
    		System.out.println("[Subsystem] Non-PID elevator: Elevating up");
    	}
    }
    
    public void stop() {
    	// stop
    	motorOne.set(0);
    	if (RobotMap.ELEVATOR_DEBUG) {
    		System.out.println("[Subsystem] Non-PID elevator: Stopped elevator");
    	}
    }
    
    public void climbDown() {
    	// full reverse
    	motorOne.set(-1);
    	if (RobotMap.ELEVATOR_DEBUG) {
    		System.out.println("[Subsystem] Non-PID elevator: Elevating down");
    	}
    }
    
    /**
     * Gets the encoder position of the elevator.
     */
    
    public double getEncoderPosition() {
    	return motorOne.getSelectedSensorPosition(0);
    }
    
    /**
     * Gets the encoder velocity of the elevator.
     */
    
    public double getEncoderVelocity() {
    	return motorOne.getSelectedSensorVelocity(0);
    }
    
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

