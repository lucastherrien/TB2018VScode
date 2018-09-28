package org.usfirst.frc.team4561.robot.commands;

import org.usfirst.frc.team4561.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * @author Snehil
 */

public class ElevatorSwitchPosition extends Command {
	
	public ElevatorSwitchPosition() {
    	requires(Robot.elevator);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.elevator.SwitchPosition();
    	
    	SmartDashboard.putNumber("Elevator Speed", Robot.elevator.getElevatorSpeed());
		SmartDashboard.putNumber("Elevator Pos", Robot.elevator.getElevatorPos());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.elevator.nearGoal();
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }

}
