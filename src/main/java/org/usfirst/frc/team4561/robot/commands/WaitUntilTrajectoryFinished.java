package org.usfirst.frc.team4561.robot.commands;

import org.usfirst.frc.team4561.robot.Robot;

import com.ctre.phoenix.motion.SetValueMotionProfile;

import edu.wpi.first.wpilibj.command.Command;

public class WaitUntilTrajectoryFinished extends Command {

	public WaitUntilTrajectoryFinished() {
	}
	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return Robot.motionProfileRunner.getSetValue() == SetValueMotionProfile.Disable;
	}
	
	protected void end(){
		System.out.println("It happened");
	}
}
