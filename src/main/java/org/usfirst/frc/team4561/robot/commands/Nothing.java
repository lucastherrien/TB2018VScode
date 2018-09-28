package org.usfirst.frc.team4561.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

public class Nothing extends Command {
//Yes, this does nothing. No, you can't delete it - it is used in automodes to give to WaitUntilPercent commands if we want to intake or outtake at specific points
	
	@Override
	protected void initialize() {
		System.out.println("Command Nothing started.");
	}
	
	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return true;
	}

}
