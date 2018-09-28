package org.usfirst.frc.team4561.robot.commands;

import org.usfirst.frc.team4561.robot.Robot;
import org.usfirst.frc.team4561.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class ToggleArmPID extends Command {

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return true;
	}
	
	protected void execute(){
		RobotMap.ARM_PID = true;
		Robot.arm.setGoal((int) Robot.arm.getEncoderPosition());
	}

}
