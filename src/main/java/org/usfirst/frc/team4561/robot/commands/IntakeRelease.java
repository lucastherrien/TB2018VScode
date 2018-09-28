package org.usfirst.frc.team4561.robot.commands;

import org.usfirst.frc.team4561.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *This is the IntakeRelease command
 *@author karth
 */
public class IntakeRelease extends Command {

    public IntakeRelease() {
        requires(Robot.intake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.intake.intakeOutHalf();
    	SmartDashboard.putNumber("IntakeRelease Encoder Position", Robot.intake.getIntakeLeftPosition());
    	SmartDashboard.putNumber("IntakeRelease Encoder Velocity", Robot.intake.getIntakeRightPosition());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
