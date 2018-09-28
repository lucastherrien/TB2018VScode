package org.usfirst.frc.team4561.robot.commands;

import org.usfirst.frc.team4561.robot.OI;
import org.usfirst.frc.team4561.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;

public class WaitUntilPositionPercentOnboard extends Command {

	double goal;
	double startPos;
	double fullPos;
	Command toRunWhenComplete;
	
	public WaitUntilPositionPercentOnboard(double percent){
		goal = percent;
	}
	
	@Override
	protected void initialize() {
		startPos = OI.ft2Units(Robot.motionProfileOnboardRunner.getCurrentTrajectory().getLeftArrayFirstPosition());
		fullPos = OI.ft2Units(Robot.motionProfileOnboardRunner.getCurrentTrajectory().getLeftArrayLastPosition());
		System.out.println("Starting at " + startPos + ", going until " + fullPos*goal + " out of " + fullPos);
	}
	
	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return Math.abs((Robot.driveTrain.getLeftPos())+startPos) > Math.abs(goal*fullPos);
	}
	protected void execute() {
		if (Math.abs(Robot.driveTrain.getLeftPos()+startPos) > Math.abs(goal*fullPos)) {
			System.out.println("It is greater than the set point, should be finishing now");
		}
		else {
			//System.out.println(Math.abs(Robot.driveTrain.getLeftPos()+startPos) + " < " + Math.abs(goal*fullPos));
		}
	}
	protected void end(){
		
		System.out.println("WaitUntilPositionPercent finished at " + Math.abs(Robot.driveTrain.getLeftPos()) + "/" + Math.abs(fullPos*goal) + " out of " + Math.abs(fullPos));
	}
}
