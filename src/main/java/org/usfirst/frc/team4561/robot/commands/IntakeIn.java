package org.usfirst.frc.team4561.robot.commands;

import org.usfirst.frc.team4561.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
/**
 *This is the IntakeIn command
 *@author Karth, Lucas
 */
public class IntakeIn extends Command {
	
	public IntakeIn(){
		requires(Robot.intake);
	}

    protected void execute(){
    	
    	
    		Robot.intake.intakeIn();
    }
    protected boolean isFinished(){
    	return true;
    }
    protected void stop(){
    }
    protected void interrupted(){
    }
}
