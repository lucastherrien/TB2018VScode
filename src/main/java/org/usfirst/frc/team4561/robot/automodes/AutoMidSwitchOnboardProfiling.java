package org.usfirst.frc.team4561.robot.automodes;

import org.usfirst.frc.team4561.robot.Robot;
import org.usfirst.frc.team4561.robot.commands.ArmReleasePosition;
import org.usfirst.frc.team4561.robot.commands.ArmVertical;
import org.usfirst.frc.team4561.robot.commands.IntakeRelease;
import org.usfirst.frc.team4561.robot.commands.IntakeStop;
import org.usfirst.frc.team4561.robot.commands.RunTrajectoryOnboard;
import org.usfirst.frc.team4561.robot.commands.SpeedGear;
import org.usfirst.frc.team4561.robot.commands.WaitUntilOnboardTrajectoryFinished;
import org.usfirst.frc.team4561.robot.commands.WaitUntilPositionPercentOnboard;
import org.usfirst.frc.team4561.trajectories.MotionProfileOnboardRunner;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 * This automode puts a block on the switch if the robot is on the left side of the field.
 * This is currently in progress.
 * @author Ben
 */
public class AutoMidSwitchOnboardProfiling extends CommandGroup {

	double delay = Robot.oi.getDashboardDelaySlider();
	
    public AutoMidSwitchOnboardProfiling() {
    	addSequential(new WaitCommand(delay));

    	addSequential(new SpeedGear());
		addSequential(new ArmVertical());
    
    	// on the left
    	if (!(Robot.switchFMSSideRight)) {
    		addSequential(new RunTrajectoryOnboard(MotionProfileOnboardRunner.TrajectorySelect.MidSwitchLeft));
    		addSequential(new WaitUntilPositionPercentOnboard(0.5));
    		addSequential(new ArmReleasePosition());
    		addSequential(new WaitUntilOnboardTrajectoryFinished());
    		addSequential(new IntakeRelease()); // drop power cube
    		addSequential(new WaitCommand(2));
    		addSequential(new IntakeStop());
    	}
    	// on the right
    	else {
    		addSequential(new RunTrajectoryOnboard(MotionProfileOnboardRunner.TrajectorySelect.MidSwitchRight));
    		addSequential(new WaitUntilPositionPercentOnboard(0.5));
    		addSequential(new ArmReleasePosition());
    		addSequential(new WaitUntilOnboardTrajectoryFinished());
    		addSequential(new IntakeRelease()); // drop power cube
    		addSequential(new WaitCommand(2));
    		addSequential(new IntakeStop());
    	}
    }
}
