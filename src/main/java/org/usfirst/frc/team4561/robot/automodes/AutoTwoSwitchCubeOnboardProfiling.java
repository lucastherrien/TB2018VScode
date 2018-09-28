package org.usfirst.frc.team4561.robot.automodes;

import org.usfirst.frc.team4561.robot.Robot;
import org.usfirst.frc.team4561.robot.commands.*;
import org.usfirst.frc.team4561.trajectories.MotionProfileOnboardRunner;
import org.usfirst.frc.team4561.trajectories.MotionProfileRunner;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class AutoTwoSwitchCubeOnboardProfiling extends CommandGroup {
double delay = Robot.oi.getDashboardDelaySlider();
	
    @SuppressWarnings("deprecation")
	public AutoTwoSwitchCubeOnboardProfiling() {
        
    	addSequential(new SpeedGear());
		addSequential(new ArmVertical());

    	// wait preassigned time
    	addSequential(new WaitCommand(delay));
    
    	// get side of switch from FMS
    	//addSequential(new CheckSwitchSide());
    	// on the left
    	if (!(Robot.switchFMSSideRight)) {
    		addSequential(new RunTrajectoryOnboard(MotionProfileOnboardRunner.TrajectorySelect.MidSwitchLeft));
    		addSequential(new WaitUntilPositionPercentOnboard(0.5));
    		addSequential(new ArmReleasePosition());
    		addSequential(new WaitUntilTrajectoryFinished());
    		addSequential(new IntakeRelease()); // drop power cubeq
    		addSequential(new WaitCommand(0.5));
    		addSequential(new IntakeStop());
    		addSequential(new ArmVertical());
    		addSequential(new RunTrajectory(MotionProfileRunner.TrajectorySelect.MidSwitchLeftReverse));
//    		addSequential(new ArmIntakePosition());
//    		addSequential(new WaitUntilOnboardTrajectoryFinished());
//    		addSequential(new RunTrajectoryOnboard(MotionProfileOnboardRunner.TrajectorySelect.MidCubePile));
//    		addSequential(new WaitUntilPositionPercentOnboard(0.5));
//    		addSequential(new IntakeIn());
//    		addSequential(new BobOpen());
//    		addSequential(new WaitUntilOnboardTrajectoryFinished());
//    		addSequential(new BobClose());
//    		addSequential(new IntakeStop());
//    		addParallel(new ArmVertical());
//    		addSequential(new RunTrajectory(MotionProfileRunner.TrajectorySelect.MidCubePileReversed));
//    		addParallel(new ArmReleasePosition());
//    		addSequential(new WaitUntilOnboardTrajectoryFinished());
//    		addSequential(new RunTrajectoryOnboard(MotionProfileOnboardRunner.TrajectorySelect.MidSwitchLeft));
////    		addSequential(new IntakeRelease());
    	}
    	// on the right
    	else {
    		addSequential(new RunTrajectoryOnboard(MotionProfileOnboardRunner.TrajectorySelect.MidSwitchRight));
    		addSequential(new WaitUntilPositionPercentOnboard(0.5));
    		addSequential(new ArmReleasePosition());
    		addSequential(new WaitUntilOnboardTrajectoryFinished());
    		addSequential(new IntakeRelease()); // drop power cubeq
    		addSequential(new WaitCommand(0.5));
    		addSequential(new IntakeStop());
    		addSequential(new ArmVertical());
    		addSequential(new RunTrajectoryOnboard(MotionProfileOnboardRunner.TrajectorySelect.MidSwitchRightReverse));
//    		addSequential(new ArmIntakePosition());
//    		addSequential(new WaitUntilOnboardTrajectoryFinished());
//    		addSequential(new RunTrajectoryOnboard(MotionProfileOnboardRunner.TrajectorySelect.MidCubePile));
//    		addSequential(new WaitUntilPositionPercentOnboard(0.5));
//    		addSequential(new IntakeIn());
//    		addSequential(new BobOpen());
//    		addSequential(new WaitUntilOnboardTrajectoryFinished());
//    		addSequential(new BobClose());
//    		addSequential(new IntakeStop());
//    		addParallel(new ArmVertical());
//    		addSequential(new RunTrajectory(MotionProfileRunner.TrajectorySelect.MidCubePileReversed));
//    		addParallel(new ArmReleasePosition());
//    		addSequential(new WaitUntilOnboardTrajectoryFinished());
//    		addSequential(new RunTrajectoryOnboard(MotionProfileOnboardRunner.TrajectorySelect.MidSwitchRight));
////    		addSequential(new IntakeRelease());
    	}
    }
}
