package org.usfirst.frc.team4561.robot.commands;

import org.usfirst.frc.team4561.robot.Robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;

/**
 * This command checks to see which side of the scale is ours.
 * This is currently in progress.
 * @author Ben
 */
public class CheckScaleSide extends Command {

    public CheckScaleSide() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	//change his so it works. this is a placeholder now.
    		//it works now
    	boolean failed = false;
    	int count = 0;
    	System.out.println(DriverStation.getInstance().getGameSpecificMessage());
    	String data = "";
    	while (data == ""){
    		data = DriverStation.getInstance().getGameSpecificMessage();
    		if (count < 1000){
    			count ++;
    		}
    		else {
    			failed = true;
    			break;
    		}
    	}
    	if (!failed)	Robot.scaleFMSSideRight = data.charAt(1) == 'R';
    	else			Robot.scaleFMSSideRight = false;
    	if (!failed)	Robot.switchFMSSideRight = data.charAt(0) == 'R';
    	else			Robot.switchFMSSideRight = false;
    	if (failed) System.out.println("Failed to get field data, assuming everything is on the left");
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
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
