package org.usfirst.frc.team4561.robot.commands;

import org.usfirst.frc.team4561.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Finishes when the current onboard trajectory is less than the specified amount of seconds away from finishing.
 * @author Kaiz
 */
public class WaitUntilTimeRemaining extends Command {

	double seconds;
	
    public WaitUntilTimeRemaining(double seconds) {
    	this.seconds = seconds;
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.motionProfileOnboardRunner.getTimeRemaining() <= seconds;
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
