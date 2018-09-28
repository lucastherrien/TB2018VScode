package org.usfirst.frc.team4561.robot.automodes;

import org.usfirst.frc.team4561.robot.Robot;
import org.usfirst.frc.team4561.robot.commands.ArmAngle;
import org.usfirst.frc.team4561.robot.commands.ArmFlat;
import org.usfirst.frc.team4561.robot.commands.ArmIntakePosition;
import org.usfirst.frc.team4561.robot.commands.ArmReleasePosition;
import org.usfirst.frc.team4561.robot.commands.ArmVertical;
import org.usfirst.frc.team4561.robot.commands.BobClose;
import org.usfirst.frc.team4561.robot.commands.BobOpen;
import org.usfirst.frc.team4561.robot.commands.DriveMagic;
import org.usfirst.frc.team4561.robot.commands.ElevatorGroundPosition;
import org.usfirst.frc.team4561.robot.commands.ElevatorScalePosition;
import org.usfirst.frc.team4561.robot.commands.IntakeIn;
import org.usfirst.frc.team4561.robot.commands.IntakeRelease;
import org.usfirst.frc.team4561.robot.commands.IntakeStop;
import org.usfirst.frc.team4561.robot.commands.RunTrajectoryOnboard;
import org.usfirst.frc.team4561.robot.commands.SpeedGear;
import org.usfirst.frc.team4561.robot.commands.TurnMagic;
import org.usfirst.frc.team4561.robot.commands.TurnToAngle;
import org.usfirst.frc.team4561.robot.commands.WaitUntilOnboardTrajectoryFinished;
import org.usfirst.frc.team4561.robot.commands.WaitUntilPositionPercent;
import org.usfirst.frc.team4561.robot.commands.WaitUntilPositionPercentOnboard;
import org.usfirst.frc.team4561.robot.commands.WaitUntilTimeElapsed;
import org.usfirst.frc.team4561.robot.commands.WaitUntilTimeRemaining;
import org.usfirst.frc.team4561.trajectories.MotionProfileOnboardRunner;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class AutoLeftTwoScaleOnboardProfiling extends CommandGroup {
double delay = Robot.oi.getDashboardDelaySlider();
	
    public AutoLeftTwoScaleOnboardProfiling() {
        
    	addSequential(new SpeedGear());
		addSequential(new ArmVertical());

    	// wait preassigned time
    	addSequential(new WaitCommand(delay));
    
    	// left side scale
    	if (!Robot.scaleFMSSideRight) {
    		addSequential(new RunTrajectoryOnboard(MotionProfileOnboardRunner.TrajectorySelect.LeftScaleLeft));
    		addSequential(new WaitUntilPositionPercentOnboard(0.5));
    		addSequential(new ElevatorScalePosition());
    		addSequential(new WaitUntilPositionPercentOnboard(0.8));
    		addSequential(new ArmAngle());
    		addSequential(new WaitUntilOnboardTrajectoryFinished());
    		addSequential(new IntakeRelease()); // drop power cube
    		addSequential(new WaitCommand(0.5));
    		addSequential(new IntakeStop());
    		addSequential(new TurnMagic(-180), 2);
    		//addSequential(new WaitUntilOnboardTrajectoryFinished());
    		addSequential(new RunTrajectoryOnboard(MotionProfileOnboardRunner.TrajectorySelect.ScaleLeftCubeLeft));
    		addSequential(new ArmVertical());
    		addSequential(new ElevatorGroundPosition());
//    		addSequential(new RunTrajectoryOnboard(MotionProfileOnboardRunner.TrajectorySelect.ScaleLeftCubeLeft2));
    		addSequential(new WaitUntilPositionPercentOnboard(0.5));
    		addSequential(new ArmIntakePosition());
    		addSequential(new BobOpen());
    		addSequential(new IntakeIn());
    		addSequential(new WaitUntilOnboardTrajectoryFinished());
    		addSequential(new BobClose());
//    		addSequential(new TurnMagic(180));
//    		addSequential(new IntakeStop());
//    		addSequential(new RunTrajectoryOnboard(MotionProfileOnboardRunner.TrajectorySelect.CubeLeftScaleLeft));
//    		addSequential(new WaitUntilPositionPercentOnboard(0));
//    		addSequential(new ElevatorScalePosition());
//    		addSequential(new WaitUntilPositionPercentOnboard(0.7));
//    		addSequential(new ArmAngle());
//    		addSequential(new WaitUntilOnboardTrajectoryFinished());
//    		addSequential(new IntakeRelease()); // drop power cube
//    		addSequential(new WaitCommand(0.5));
//    		addSequential(new IntakeStop());
//    		addSequential(new DriveMagic(-40, -40));
//    		addSequential(new ArmVertical());
//    		addSequential(new ElevatorGroundPosition());
    	}
    	// right side scale
    	else if (Robot.scaleFMSSideRight) {
    		addSequential(new RunTrajectoryOnboard(MotionProfileOnboardRunner.TrajectorySelect.LeftScaleRight));
    		addSequential(new WaitUntilTimeRemaining(1.7));
    		addSequential(new ElevatorScalePosition());
    		addSequential(new WaitUntilTimeRemaining(0.3));
    		addSequential(new ArmAngle());
    		addSequential(new WaitUntilOnboardTrajectoryFinished());
    		addSequential(new IntakeRelease()); // drop power cube
    		addSequential(new WaitCommand(0.5));
    		addSequential(new IntakeStop());
    		addSequential(new ArmVertical());
    		addSequential(new ElevatorGroundPosition());
    		addSequential(new TurnToAngle(180));
    		addSequential(new RunTrajectoryOnboard(MotionProfileOnboardRunner.TrajectorySelect.ScaleRightCubeRight));
    		addSequential(new WaitUntilPositionPercentOnboard(0.8));
    		addSequential(new ArmIntakePosition());
    		addSequential(new BobOpen());
    		addSequential(new IntakeIn());
    		addSequential(new WaitUntilOnboardTrajectoryFinished());
    		addSequential(new BobClose());
    		addSequential(new TurnToAngle(20));
    		addSequential(new IntakeStop());
    		addSequential(new RunTrajectoryOnboard(MotionProfileOnboardRunner.TrajectorySelect.CubeRightScaleRight));
    		addSequential(new WaitUntilTimeRemaining(1.7));
    		addSequential(new ElevatorScalePosition());
    		addSequential(new WaitUntilTimeRemaining(0.3));
    		addSequential(new ArmAngle());
    		addSequential(new WaitUntilOnboardTrajectoryFinished());
    		addSequential(new IntakeRelease()); // drop power cube
    		addSequential(new WaitCommand(0.5));
    		addSequential(new IntakeStop());
    		addSequential(new DriveMagic(-40, -40));
    		addSequential(new ArmVertical());
    		addSequential(new ElevatorGroundPosition());
    	}
    }
}
