package org.usfirst.frc.team4561.robot.commands;

import org.usfirst.frc.team4561.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class ResetArm extends Command {

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return true;
	}
	
	protected void execute(){
		if (Robot.oi.getControllerLeftY()==0) Robot.arm.resetFlow();
//		else Robot.arm.resetFlow();
	}

}
