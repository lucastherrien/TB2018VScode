package org.usfirst.frc.team4561.robot.commands;

import org.usfirst.frc.team4561.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Finishes when the specified amount of seconds have elapsed in the current onboard trajectory.
 * @author Kaiz
 */
public class WaitUntilTimeElapsed extends Command {

	double seconds;
	
    public WaitUntilTimeElapsed(double seconds) {
    	this.seconds = seconds;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	setTimeout(0);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.motionProfileOnboardRunner.getTimeElapsed() >= seconds;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
