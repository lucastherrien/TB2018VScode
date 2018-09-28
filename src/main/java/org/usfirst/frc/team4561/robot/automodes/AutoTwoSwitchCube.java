package org.usfirst.frc.team4561.robot.automodes;

import org.usfirst.frc.team4561.robot.Robot;
import org.usfirst.frc.team4561.robot.commands.*;
import org.usfirst.frc.team4561.trajectories.MotionProfileRunner;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.command.WaitUntilCommand;

public class AutoTwoSwitchCube extends CommandGroup {
double delay = Robot.oi.getDashboardDelaySlider();
	
    @SuppressWarnings("deprecation")
	public AutoTwoSwitchCube() {
        
    	addSequential(new SpeedGear());
		addSequential(new ArmVertical());

    	// wait preassigned time
    	addSequential(new WaitCommand(delay));
    
    	// get side of switch from FMS
    	//addSequential(new CheckSwitchSide());
    	// on the left
    	if (!(Robot.switchFMSSideRight)) {
    		addSequential(new RunTrajectory(MotionProfileRunner.TrajectorySelect.MidSwitchLeft));
    		addParallel(new WaitUntilPositionPercent(0.1, new ArmReleasePosition()));
    		addSequential(new WaitUntilTrajectoryFinished());
    		addSequential(new IntakeRelease()); // drop power cubeq
    		addSequential(new WaitCommand(0.5));
    		addSequential(new IntakeStop());
    		addParallel(new ArmReleasePosition());
    		addSequential(new RunTrajectory(MotionProfileRunner.TrajectorySelect.MidSwitchLeftReverse));
    		addSequential(new WaitUntilTrajectoryFinished());
    		addSequential(new ArmIntakePosition());
    		addParallel(new RunTrajectory(MotionProfileRunner.TrajectorySelect.MidCubePile));
    		addSequential(new BobOpen());
    		addSequential(new IntakeIn());
    		addSequential(new WaitUntilPositionPercent(0.99, new Nothing()));
    		addSequential(new BobClose());
    		addSequential(new WaitCommand(0.25));
    		addSequential(new IntakeStop());
    		addSequential(new WaitCommand(0.25));
    		addSequential(new ArmReleasePosition());
    		addSequential(new RunTrajectory(MotionProfileRunner.TrajectorySelect.MidCubePileReversed));
    		addSequential(new WaitUntilPositionPercent(0.99, new Nothing()));
//    		addParallel(new ArmReleasePosition());
//    		addSequential(new RunTrajectory(MotionProfileRunner.TrajectorySelect.MidSwitchLeft));
//    		addSequential(new WaitUntilTrajectoryFinished());
//    		addSequential(new IntakeRelease());
    	}
    	// on the right
    	else {
    		addSequential(new RunTrajectory(MotionProfileRunner.TrajectorySelect.MidSwitchRight));
    		addParallel(new WaitUntilPositionPercent(0.1, new ArmReleasePosition()));
    		addSequential(new WaitUntilTrajectoryFinished());
    		addSequential(new IntakeRelease()); // drop power cubeq
    		addSequential(new WaitCommand(0.5));
    		addSequential(new IntakeStop());
    		addParallel(new ArmReleasePosition());
    		addSequential(new RunTrajectory(MotionProfileRunner.TrajectorySelect.MidSwitchRightReverse));
    		addSequential(new WaitUntilPositionPercent(0.99, new Nothing()));
    		addSequential(new ArmIntakePosition());
    		addParallel(new RunTrajectory(MotionProfileRunner.TrajectorySelect.MidCubePile));
    		addSequential(new IntakeIn());
    		addSequential(new BobOpen());
    		addSequential(new WaitUntilPositionPercent(0.99, new Nothing()));
    		addSequential(new BobClose());
    		addSequential(new WaitCommand(0.25));
    		addSequential(new IntakeStop());
    		addSequential(new WaitCommand(0.25));
    		addSequential(new ArmReleasePosition());
    		addSequential(new RunTrajectory(MotionProfileRunner.TrajectorySelect.MidCubePileReversed));
    		addSequential(new WaitUntilPositionPercent(0.99, new Nothing()));
//    		addParallel(new ArmReleasePosition());
//    		addSequential(new RunTrajectory(MotionProfileRunner.TrajectorySelect.MidSwitchRight));
//    		addSequential(new WaitUntilTrajectoryFinished());
//    		addSequential(new IntakeRelease());
    	}
    }
}
