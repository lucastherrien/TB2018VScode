package org.usfirst.frc.team4561.robot.automodes;

import org.usfirst.frc.team4561.robot.Robot;
import org.usfirst.frc.team4561.robot.commands.ArmAngle;
import org.usfirst.frc.team4561.robot.commands.ArmFlat;
import org.usfirst.frc.team4561.robot.commands.ArmReleasePosition;
import org.usfirst.frc.team4561.robot.commands.ArmVertical;
import org.usfirst.frc.team4561.robot.commands.BobOpen;
import org.usfirst.frc.team4561.robot.commands.DriveMagic;
import org.usfirst.frc.team4561.robot.commands.ElevatorGroundPosition;
import org.usfirst.frc.team4561.robot.commands.ElevatorScalePosition;
import org.usfirst.frc.team4561.robot.commands.IntakeOutFull;
import org.usfirst.frc.team4561.robot.commands.IntakeRelease;
import org.usfirst.frc.team4561.robot.commands.IntakeStop;
import org.usfirst.frc.team4561.robot.commands.RunTrajectoryOnboard;
import org.usfirst.frc.team4561.robot.commands.SpeedGear;
import org.usfirst.frc.team4561.robot.commands.WaitUntilOnboardTrajectoryFinished;
import org.usfirst.frc.team4561.robot.commands.WaitUntilPositionPercentOnboard;
import org.usfirst.frc.team4561.trajectories.MotionProfileOnboardRunner;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class RightAlwaysLeftScale extends CommandGroup {
double delay = Robot.oi.getDashboardDelaySlider();
	
    public RightAlwaysLeftScale() {
        
    	addSequential(new SpeedGear());
//		addSequential(new ArmVertical());

    	// wait preassigned time
    	addSequential(new WaitCommand(delay));
    
    	// on the right
    	
    		addSequential(new RunTrajectoryOnboard(MotionProfileOnboardRunner.TrajectorySelect.RightScaleLeftZoom));
//    		addSequential(new WaitUntilPositionPercentOnboard(0.5));
//    		addSequential(new ElevatorScalePosition());
//    		addSequential(new ArmAngle());
//    		addSequential(new WaitUntilOnboardTrajectoryFinished());
//    		addSequential(new IntakeRelease()); // drop power cubeq
//    		addSequential(new WaitCommand(0.5));
//    		addSequential(new IntakeStop());
//    		addSequential(new DriveMagic(-40, -40));
//    		addSequential(new ArmVertical());
//    		addSequential(new ElevatorGroundPosition());

    }
}
