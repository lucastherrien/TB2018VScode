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
import org.usfirst.frc.team4561.robot.commands.WaitUntilOnboardTrajectoryFinished;
import org.usfirst.frc.team4561.robot.commands.WaitUntilPositionPercentOnboard;
import org.usfirst.frc.team4561.trajectories.MotionProfileOnboardRunner;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class AutoLeftScaleOnboardProfilingNearOnly extends CommandGroup {
double delay = Robot.oi.getDashboardDelaySlider();
	
    public AutoLeftScaleOnboardProfilingNearOnly() {
        
    	addSequential(new SpeedGear());
		addSequential(new ArmVertical());

    	// wait preassigned time
    	addSequential(new WaitCommand(delay));
    
    	// on the left
    	if (!(Robot.scaleFMSSideRight)) {
    		addSequential(new RunTrajectoryOnboard(MotionProfileOnboardRunner.TrajectorySelect.LeftScaleLeft));
    		addSequential(new WaitUntilPositionPercentOnboard(0.35));
    		addSequential(new ElevatorScalePosition());
    		addSequential(new WaitUntilPositionPercentOnboard(0.9));
    		addSequential(new ArmAngle());
    		addSequential(new WaitUntilOnboardTrajectoryFinished());
    		addSequential(new IntakeRelease()); // drop power cube
    		addSequential(new WaitCommand(0.5));
    		addSequential(new IntakeStop());
    		
    		addSequential(new ArmVertical());
//    		addSequential(new TurnMagic(-180), 2);
//    		addSequential(new ElevatorGroundPosition());
//    		addSequential(new ArmIntakePosition());
//    		addSequential(new BobOpen());
//    		addSequential(new IntakeIn());
//    		addSequential(new RunTrajectoryOnboard(MotionProfileOnboardRunner.TrajectorySelect.CubeLeftScaleLeft));
//    		addSequential(new WaitUntilOnboardTrajectoryFinished());
//    		addSequential(new BobClose());
//    		addSequential(new WaitCommand(0.25));
//    		addSequential(new IntakeStop());
//    		addSequential(new WaitCommand(0.25));
//    		addSequential(new ArmVertical());
//    		addSequential(new TurnMagic(-180), 2);
//    		addSequential(new WaitCommand(0.25));
//    		addSequential(new ElevatorScalePosition());
//    		addSequential(new DriveMagic(40, 40));
//    		addSequential(new ArmAngle());
//    		addSequential(new WaitCommand(0.25));
//    		addSequential(new IntakeRelease());
//    		addSequential(new WaitCommand(0.5));
//    		addSequential(new IntakeStop());
    		
    		addSequential(new DriveMagic(-40, -40));
    		addSequential(new ArmVertical());
    		addSequential(new ElevatorGroundPosition());
    	}
    	// on the right
    	else {
//    		addSequential(new RunTrajectoryOnboard(MotionProfileOnboardRunner.TrajectorySelect.LeftScaleRight));
//    		addSequential(new WaitUntilPositionPercentOnboard(0.6));
//    		addSequential(new ElevatorScalePosition());
//    		addSequential(new WaitUntilPositionPercentOnboard(0.9));
//    		addSequential(new ArmAngle());
//    		addSequential(new WaitUntilOnboardTrajectoryFinished());
//    		addSequential(new IntakeRelease()); // drop power cube
//    		addSequential(new WaitCommand(0.5));
//    		addSequential(new IntakeStop());
//    		addSequential(new DriveMagic(-40, -40));
//    		addSequential(new ArmVertical());
//    		addSequential(new ElevatorGroundPosition());
    		addSequential(new AutoDriveToLine());
    	}
    }
}
