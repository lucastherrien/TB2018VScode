package org.usfirst.frc.team4561.robot.commands;

import org.usfirst.frc.team4561.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class ArmVertical extends Command {

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return Robot.arm.nearGoal();
	}
	
	protected void initialize(){
		Robot.arm.UpPostition();
	}

}
