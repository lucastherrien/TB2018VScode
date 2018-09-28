package org.usfirst.frc.team4561.robot.commands;

import org.usfirst.frc.team4561.robot.OI;
import org.usfirst.frc.team4561.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;

public class WaitUntilPositionPercent extends Command {

	double goal;
	double startPos;
	double fullPos;
	Command toRunWhenComplete;
	
	public WaitUntilPositionPercent(double percent, Command toRunWhenComplete) {
		goal = percent;
		this.toRunWhenComplete = toRunWhenComplete;
	}
	
	@Deprecated
	public WaitUntilPositionPercent(double percent, double start, double end, Command toRunWhenComplete){
		goal = percent;
		startPos = OI.ft2Units(start);
		fullPos = OI.ft2Units(end);
		this.toRunWhenComplete = toRunWhenComplete;
		System.out.println("Starting at " + startPos + ", going until " + fullPos*percent + " out of " + fullPos);
		Robot.driveTrain.resetEncoders();
	}
	
	@Override
	protected void initialize() {
		startPos = OI.ft2Units(Robot.motionProfileRunner.getCurrentTrajectory().getLeftArrayFirstPosition());
		fullPos = OI.ft2Units(Robot.motionProfileRunner.getCurrentTrajectory().getLeftArrayLastPosition());
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
		
		toRunWhenComplete.start();
	}
}