package org.usfirst.frc.team4561.robot.commands;

import org.usfirst.frc.team4561.trajectories.MotionProfileOnboardRunner;
import org.usfirst.frc.team4561.robot.Robot;
import org.usfirst.frc.team4561.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RunTrajectoryOnboard extends Command {

	MotionProfileOnboardRunner.TrajectorySelect trajectory;
	
    public RunTrajectoryOnboard(MotionProfileOnboardRunner.TrajectorySelect trajectory) {
    	this.trajectory = trajectory;
    	requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("Starting trajectory: " + trajectory.toString());
//    	Robot.gyro.reset();
    	Robot.motionProfileOnboardRunner.prepare();
    	Robot.motionProfileOnboardRunner.setCurrentTrajectory(trajectory);
    	Robot.motionProfileOnboardRunner.startMotionProfile();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true; // Robot.motionProfileOnboardRunner.isFinished();
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("Finished trajectory: " + trajectory.toString());
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
