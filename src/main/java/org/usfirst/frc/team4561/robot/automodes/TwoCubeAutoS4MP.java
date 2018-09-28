package org.usfirst.frc.team4561.robot.automodes;

import org.usfirst.frc.team4561.robot.Robot;
import org.usfirst.frc.team4561.robot.commands.ArmIntakePosition;
import org.usfirst.frc.team4561.robot.commands.ArmReleasePosition;
import org.usfirst.frc.team4561.robot.commands.DriveProfile;
import org.usfirst.frc.team4561.robot.commands.ElevatorGroundPosition;
import org.usfirst.frc.team4561.robot.commands.ElevatorScalePosition;
import org.usfirst.frc.team4561.robot.commands.ElevatorSwitchPosition;
import org.usfirst.frc.team4561.robot.commands.IntakeIn;
import org.usfirst.frc.team4561.robot.commands.IntakeOutFull;
import org.usfirst.frc.team4561.robot.commands.WaitUntilPositionPercent;

import edu.wpi.first.wpilibj.command.CommandGroup;
import jaci.pathfinder.Trajectory;
/**
 * Right Scale then Right switch starting from Right.
 * @author Max
 */
public class TwoCubeAutoS4MP extends CommandGroup {
	@SuppressWarnings("deprecation")
	public TwoCubeAutoS4MP(){
		 // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
		Trajectory pointsR = Robot.rightScaleRight.getRightTrajectory();
		Trajectory pointsL = Robot.rightScaleRight.getLeftTrajectory();
		double start = pointsL.get(0).position;
		double end = pointsL.get(pointsL.length()-1).position;
		addSequential(new DriveProfile(pointsR, pointsL));
		addSequential(new WaitUntilPositionPercent(0.5, start, end, new ElevatorScalePosition()));
		addSequential(new ArmReleasePosition());
    	addSequential(new IntakeOutFull());
		pointsR = Robot.rightScaleTurnAroundS4.getRightTrajectory();
		pointsL = Robot.rightScaleTurnAroundS4.getLeftTrajectory();
		addSequential(new DriveProfile(pointsR, pointsL));
		addSequential(new WaitUntilPositionPercent(0.5, start, end, new ElevatorGroundPosition()));
		addSequential(new ArmIntakePosition());
		pointsR = Robot.scaleRightSwitchRightCube.getRightTrajectory();
		pointsL = Robot.scaleRightSwitchRightCube.getLeftTrajectory();
		addSequential(new DriveProfile(pointsR, pointsL));
		addSequential(new WaitUntilPositionPercent(0.5, start, end, new IntakeIn()));
		addSequential(new ElevatorSwitchPosition());
		addSequential(new ArmReleasePosition());
		addSequential(new IntakeOutFull());
	}
}
