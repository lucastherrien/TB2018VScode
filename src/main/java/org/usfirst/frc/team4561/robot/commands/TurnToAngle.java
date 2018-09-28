package org.usfirst.frc.team4561.robot.commands;

import org.usfirst.frc.team4561.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PIDCommand;

/**
 *
 */
public class TurnToAngle extends PIDCommand {

	static double kP = 0.003;
	static double kI = 0;
	static double kD = 0;
	
	double angle;
	
    public TurnToAngle(double angle) {
    	super(kP, kI, kD);
    	requires(Robot.driveTrain);
		//Set input and output restraints
		setInputRange(-180.0, 180.0);
		getPIDController().setContinuous(true);
		getPIDController().setAbsoluteTolerance(1.0);
		this.angle = angle;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	setSetpoint(angle);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return getPIDController().onTarget();
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }

	@Override
	protected double returnPIDInput() {
		// TODO Auto-generated method stub
		return Robot.gyro.getYaw();
	}

	@Override
	protected void usePIDOutput(double output) {
		Robot.driveTrain.frontLeft.set(ControlMode.PercentOutput, output);
		Robot.driveTrain.frontRight.set(ControlMode.PercentOutput, -output);
	}
}
