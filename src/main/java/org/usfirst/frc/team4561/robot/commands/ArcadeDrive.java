package org.usfirst.frc.team4561.robot.commands;

import org.usfirst.frc.team4561.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class ArcadeDrive extends Command {
	
	public ArcadeDrive() {
		requires(Robot.driveTrain);
	}
	
	protected void execute() {
		Robot.driveTrain.arcadeDrive(Robot.oi.getLeftStickY(), Robot.oi.getRightStickX());
	}	  
	  
	protected void initialize() {}
	  
	protected void end() {}
	  
	protected void interrupted() {
		end();
	}
    
	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}

}
