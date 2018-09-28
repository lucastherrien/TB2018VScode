package org.usfirst.frc.team4561.robot.commands;

import org.usfirst.frc.team4561.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;

public class WaitUntilOnboardTrajectoryFinished extends Command {

	public WaitUntilOnboardTrajectoryFinished() {
	}
	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return Robot.motionProfileOnboardRunner.isFinished();
	}
	
	protected void end(){
		System.out.println("It happened");
	}
}
