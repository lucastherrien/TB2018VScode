package org.usfirst.frc.team4561.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team4561.robot.Robot;
import org.usfirst.frc.team4561.robot.RobotMap;

/**
 * This command drives in tank mode for a set time.
 * @author Ben
 */
public class TankDriveTimed extends Command {

	double leftPower;
	double rightPower;
	double time;
	
    public TankDriveTimed(double left, double right, double seconds) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveTrain);
    	this.leftPower = left;
    	this.rightPower = right;
    	this.time = seconds;
    	setTimeout(seconds);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        if (RobotMap.DRIVETRAIN_DEBUG) {
        	System.out.println("[Command] Intializing TankDriveTimed for " + time + " seconds. Left: " + leftPower + ", right: " + rightPower);
        }
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.driveTrain.tankDrive(leftPower, rightPower);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
        if (RobotMap.DRIVETRAIN_DEBUG) {
        	System.out.println("[Command] Stopping TankDriveTimed");
        }
        Robot.driveTrain.tankDrive(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
