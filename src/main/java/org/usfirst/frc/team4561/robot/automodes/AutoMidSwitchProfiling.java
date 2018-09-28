package org.usfirst.frc.team4561.robot.automodes;

import org.usfirst.frc.team4561.robot.Robot;
import org.usfirst.frc.team4561.robot.commands.ArmReleasePosition;
import org.usfirst.frc.team4561.robot.commands.ArmVertical;
import org.usfirst.frc.team4561.robot.commands.IntakeRelease;
import org.usfirst.frc.team4561.robot.commands.IntakeStop;
import org.usfirst.frc.team4561.robot.commands.ResetDrive;
import org.usfirst.frc.team4561.robot.commands.RunTrajectory;
import org.usfirst.frc.team4561.robot.commands.SpeedGear;
import org.usfirst.frc.team4561.robot.commands.WaitUntilPositionPercent;
import org.usfirst.frc.team4561.robot.commands.WaitUntilTrajectoryFinished;
import org.usfirst.frc.team4561.trajectories.MotionProfileRunner;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 * This automode puts a block on the switch if the robot is on the left side of the field.
 * This is currently in progress.
 * @author Ben
 */
public class AutoMidSwitchProfiling extends CommandGroup {

	double delay = Robot.oi.getDashboardDelaySlider();
	
    public AutoMidSwitchProfiling() {
    	addSequential(new WaitCommand(delay));

    	addSequential(new SpeedGear());
		addSequential(new ArmVertical());
		addSequential(new ResetDrive());
    
    	// on the left
    	if (!(Robot.switchFMSSideRight)) {
    		addSequential(new RunTrajectory(MotionProfileRunner.TrajectorySelect.MidSwitchLeft));
    		addParallel(new WaitUntilPositionPercent(0.5, new ArmReleasePosition()));
    		addSequential(new WaitUntilTrajectoryFinished());
    		addSequential(new IntakeRelease()); // drop power cubeq
    		addSequential(new WaitCommand(5));
    		addSequential(new IntakeStop());
    	}
    	// on the right
    	else {
    		addSequential(new RunTrajectory(MotionProfileRunner.TrajectorySelect.MidSwitchRight));
    		addParallel(new WaitUntilPositionPercent(0.5, new ArmReleasePosition()));
    		addSequential(new WaitUntilTrajectoryFinished());
    		addSequential(new IntakeRelease()); // drop power cubeq
    		addSequential(new WaitCommand(5));
    		addSequential(new IntakeStop());
    	}
    }
}
