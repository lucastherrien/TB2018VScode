package org.usfirst.frc.team4561.robot.automodes;

import org.usfirst.frc.team4561.robot.Robot;
import org.usfirst.frc.team4561.robot.commands.ArmReleasePosition;
import org.usfirst.frc.team4561.robot.commands.ArmVertical;
import org.usfirst.frc.team4561.robot.commands.DriveMagic;
import org.usfirst.frc.team4561.robot.commands.ElevatorGroundPosition;
import org.usfirst.frc.team4561.robot.commands.ElevatorScalePosition;
import org.usfirst.frc.team4561.robot.commands.IntakeRelease;
import org.usfirst.frc.team4561.robot.commands.IntakeStop;
import org.usfirst.frc.team4561.robot.commands.ResetDrive;
import org.usfirst.frc.team4561.robot.commands.RunTrajectory;
import org.usfirst.frc.team4561.robot.commands.SpeedGear;
import org.usfirst.frc.team4561.robot.commands.WaitUntilPositionPercent;
import org.usfirst.frc.team4561.trajectories.MotionProfileRunner;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class AutoRightScaleProfiling extends CommandGroup {
double delay = Robot.oi.getDashboardDelaySlider();
	
    public AutoRightScaleProfiling() {
        
    	addSequential(new SpeedGear());
		addSequential(new ArmVertical());
		addSequential(new ResetDrive());

    	// wait preassigned time
    	addSequential(new WaitCommand(delay));
    
    	// on the right
    	if (Robot.scaleFMSSideRight) {
    		addParallel(new RunTrajectory(MotionProfileRunner.TrajectorySelect.RightScaleRight));
    		addParallel(new WaitUntilPositionPercent(0.5, new ElevatorScalePosition()));
    		addParallel(new WaitUntilPositionPercent(0.9, new ArmReleasePosition()));
    		addSequential(new IntakeRelease()); // drop power cubeq
    		addSequential(new WaitCommand(0.5));
    		addSequential(new IntakeStop());
    		addSequential(new DriveMagic(-20, -20));
    		addSequential(new ArmVertical());
    		addSequential(new ElevatorGroundPosition());
    	}
    	// on the left
    	else {
    		addParallel(new RunTrajectory(MotionProfileRunner.TrajectorySelect.RightScaleLeft));
    		addParallel(new WaitUntilPositionPercent(0.5, new ElevatorScalePosition()));
    		addParallel(new WaitUntilPositionPercent(0.9, new ArmReleasePosition()));
    		addSequential(new IntakeRelease()); // drop power cubeq
    		addSequential(new WaitCommand(0.5));
    		addSequential(new IntakeStop());
    		addSequential(new DriveMagic(-20, -20));
    		addSequential(new ArmVertical());
    		addSequential(new ElevatorGroundPosition());
    	}
    }
}
