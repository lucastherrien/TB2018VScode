package org.usfirst.frc.team4561.robot.automodes;

import org.usfirst.frc.team4561.robot.commands.Nothing;
import org.usfirst.frc.team4561.robot.commands.RunTrajectoryOnboard;
import org.usfirst.frc.team4561.robot.commands.SpeedGear;
import org.usfirst.frc.team4561.robot.commands.WaitUntilPositionPercentOnboard;
import org.usfirst.frc.team4561.robot.commands.WaitUntilTrajectoryFinished;
import org.usfirst.frc.team4561.trajectories.MotionProfileOnboardRunner;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoTestOnboardProfiling extends CommandGroup {
	public AutoTestOnboardProfiling(){
		addSequential(new SpeedGear());
		addSequential(new RunTrajectoryOnboard(MotionProfileOnboardRunner.TrajectorySelect.TestTrajectory));
		addSequential(new WaitUntilPositionPercentOnboard(0.5));
		addSequential(new WaitUntilTrajectoryFinished());
		addSequential(new Nothing());
	}
}
