/**
 * Example logic for firing and managing motion profiles.
 * This example sends MPs, waits for them to finish
 * Although this code uses a CANTalon, nowhere in this module do we changeMode() or call set() to change the output.
 * This is done in Robot.java to demonstrate how to change control modes on the fly.
 * 
 * The only routines we call on Talon are....
 * 
 * changeMotionControlFramePeriod
 * 
 * getMotionProfileStatus		
 * clearMotionProfileHasUnderrun     to get status and potentially clear the error flag.
 * 
 * pushMotionProfileTrajectory
 * clearMotionProfileTrajectories
 * processMotionProfileBuffer,   to push/clear, and process the trajectory points.
 * 
 * getControlMode, to check if we are in Motion Profile Control mode.
 * 
 * Example of advanced features not demonstrated here...
 * [1] Calling pushMotionProfileTrajectory() continuously while the Talon executes the motion profile, thereby keeping it going indefinitely.
 * [2] Instead of setting the sensor position to zero at the start of each MP, the program could offset the MP's position based on current position. 
 */
package org.usfirst.frc.team4561.trajectories;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;

import org.usfirst.frc.team4561.robot.OI;
import org.usfirst.frc.team4561.robot.Robot;
import org.usfirst.frc.team4561.robot.RobotMap;

import com.ctre.phoenix.motion.*;
import com.ctre.phoenix.motion.TrajectoryPoint.TrajectoryDuration;

/**
 * Based on https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java/MotionProfile/src/org/usfirst/frc/team217/robot/MotionProfileExample.java
 * @author Kaiz
 *
 */
public class MotionProfileRunner {
	
	public static Path leftScaleLeft = new LeftScaleLeft();
	public static Path leftScaleRight = new LeftScaleRight();
	public static Path leftScaleTurnAroundS3 = new LeftScaleTurnAroundS3();
	public static Path midSwitchLeft = new MidSwitchLeftOffboard();
	public static Path midSwitchRight = new MidSwitchRightOffboard();
	public static Path rightScaleLeft = new RightScaleLeft();
	public static Path rightScaleRight = new RightScaleRight();
	public static Path rightScaleTurnAroundS4 = new RightScaleTurnAroundS4();
	public static Path scaleLeftSwitchLeftCube = new ScaleLeftSwitchLeftCube();
	public static Path scaleLeftTurnAround = new ScaleLeftTurnAround();
	public static Path scaleRightSwitchRightCube = new ScaleRightSwitchRightCube();
	public static Path scaleRightTurnAround = new ScaleRightTurnAround();
	public static Path midSwitchLeftReverse = new MidSwitchLeftReverse();
	public static Path midCubePile = new MidCubePile();
	public static Path midCubePileReverse = new MidCubePileReverse();
	public static Path midSwitchRightReverse = new MidSwitchRightReverse();
	public static Path testTrajectory = new TestTrajectory();
	
	/**
	 * All the different trajectories the robot can run.
	 */
	public enum TrajectorySelect {
		LeftScaleLeft(leftScaleLeft),
		LeftScaleRight(leftScaleRight),
		LeftScaleTurnAroundS3(leftScaleTurnAroundS3),
		MidSwitchLeft(midSwitchLeft),
		MidSwitchRight(midSwitchRight),
		RightScaleLeft(rightScaleLeft),
		RightScaleRight(rightScaleRight),
		RightScaleTurnAroundS4(rightScaleTurnAroundS4),
		ScaleLeftSwitchLeftCube(scaleLeftSwitchLeftCube),
		ScaleLeftTurnAround(scaleLeftTurnAround),
		ScaleRightSwitchRightCube(scaleRightSwitchRightCube),
		ScaleRightTurnAround(scaleRightTurnAround),
		MidSwitchLeftReverse(midSwitchLeftReverse),
		MidCubePile(midCubePile),
		MidCubePileReversed(midCubePileReverse),
		MidSwitchRightReverse(midSwitchRightReverse),
		TestTrajectory(testTrajectory);
		
		Path trajectory;
		TrajectorySelect(Path trajectory) {
			this.trajectory = trajectory;
		}
		/**
		 * @return An array of waypoints for the left side of the trajectory, each waypoint containing position, velocity, and time step. 
		 */
		private double[][] getLeftArray() {
			return trajectory.getLeftArray();
		}
		/**
		 * @return An array of waypoints for the right side of the trajectory, each waypoint containing position, velocity, and time step. 
		 */
		private double[][] getRightArray() {
			return trajectory.getRightArray();
		}
		/**
		 * @return The number of waypoints in the trajectory. This will be the length of the left and right arrays.
		 */
		private int getCount() {
			return trajectory.getCount();
		}
		/**
		 * @return Whether or not the robot goes backwards for this trajectory.
		 */
		private boolean isReversed() {
			return trajectory.isReversed();
		}
		/**
		 * @return The first position in the left trajectory.
		 */
		public double getLeftArrayFirstPosition() {
			return trajectory.getLeftArray()[0][0];
		}
		/**
		 * @return The last position in the left trajectory.
		 */
		public double getLeftArrayLastPosition() {
			return trajectory.getLeftArray()[trajectory.getCount()-1][0];
		}
	}
	
	/**
	 * The trajectory we are running right now.
	 */
	private TrajectorySelect currentTrajectory;
	
	/**
	 * Sets the trajectory that the robot should run next.
	 * @param traj
	 */
	public void setCurrentTrajectory(TrajectorySelect traj) {
		currentTrajectory = traj;
	}
	/**
	 * Gets the trajectory that the robot is ready to run next.
	 * @param traj
	 */
	public TrajectorySelect getCurrentTrajectory() {
		return currentTrajectory;
	}
	
	/**
	 * The status of the motion profile executer and buffer inside the Talon.
	 * Instead of creating a new one every time we call getMotionProfileStatus,
	 * keep one copy.
	 */
	private MotionProfileStatus leftStatus = new MotionProfileStatus();
	private MotionProfileStatus rightStatus = new MotionProfileStatus();

	/** Additional cache for holding the active trajectory points */
	double leftPos = 0, leftVel = 0, leftHeading = 0, rightPos = 0, rightVel = 0, rightHeading = 0;

	/**
	 * Reference to the talons we plan on manipulating. We will not changeMode()
	 * or call set(), just get motion profile status and make decisions based on
	 * motion profile.
	 */
	private TalonSRX leftTalon;
	private TalonSRX rightTalon;
	
	/**
	 * State machine to make sure we let enough of the motion profile stream to
	 * talon before we fire it.
	 */
	private int state = 0;
	/**
	 * Any time you have a state machine that waits for external events, its a
	 * good idea to add a timeout. Set to -1 to disable. Set to nonzero to count
	 * down to '0' which will print an error message. Counting loops is not a
	 * very accurate method of tracking timeout, but this is just conservative
	 * timeout. Getting time-stamps would certainly work too, this is just
	 * simple (no need to worry about timer overflows).
	 */
	private int loopTimeout = -1;
	/**
	 * If start() gets called, this flag is set and in the control() we will
	 * service it.
	 */
	private boolean start = false;

	/**
	 * Since the CANTalon.set() routine is mode specific, deduce what we want
	 * the set value to be and let the calling module apply it whenever we
	 * decide to switch to MP mode.
	 */
	private SetValueMotionProfile setValue = SetValueMotionProfile.Disable;
	/**
	 * How many trajectory points do we wait for before firing the motion
	 * profile.
	 */
	private static final int kMinPointsInTalon = 5;
	/**
	 * Just a state timeout to make sure we don't get stuck anywhere. Each loop
	 * is about 20ms.
	 */
	private static final int kNumLoopsTimeout = 10;
	
	/**
	 * Lets create a periodic task to funnel our trajectory points into our talon.
	 * It doesn't need to be very accurate, just needs to keep pace with the motion
	 * profiler executer.  Now if you're trajectory points are slow, there is no need
	 * to do this, just call _talon.processMotionProfileBuffer() in your teleop loop.
	 * Generally speaking you want to call it at least twice as fast as the duration
	 * of your trajectory points.  So if they are firing every 20ms, you should call 
	 * every 10ms.
	 */
	class PeriodicRunnable implements java.lang.Runnable {
	    public void run() {
	    	leftTalon.processMotionProfileBuffer();
	    	rightTalon.processMotionProfileBuffer();
	    }
	}
	Notifier notifier = new Notifier(new PeriodicRunnable());
	
	/**
	 * @param leftTalon reference to left master TalonSRX object.
	 * @param rightTalon reference to the right master TalonSRX object.
	 * @param leftArray array containing the left motion profile
	 * @param rightArray array containing the right motion profile
	 */
	public MotionProfileRunner(TalonSRX leftTalon, TalonSRX rightTalon) {
		this.leftTalon = leftTalon;
		this.rightTalon = rightTalon;
		/*
		 * Set the control frame rate and the notifer period.
		 * Period should be less than half of time per point.
		 */
		leftTalon.changeMotionControlFramePeriod(2);
		rightTalon.changeMotionControlFramePeriod(2);
		notifier.startPeriodic(0.002);
	}

	/**
	 * Called to clear Motion profile buffer and reset state info during
	 * disabled and when Talon is not in MP control mode.
	 */
	public void reset() {
		/*
		 * Let's clear the buffer just in case user decided to disable in the
		 * middle of an MP, and now we have the second half of a profile just
		 * sitting in memory.
		 */
		leftTalon.clearMotionProfileTrajectories();
		rightTalon.clearMotionProfileTrajectories();
		Robot.driveTrain.invertLeftSide(RobotMap.LEFT_SIDE_INVERTED);
		Robot.driveTrain.invertRightSide(RobotMap.RIGHT_SIDE_INVERTED);
		/* When we do re-enter motionProfile control mode, stay disabled. */
		setValue = SetValueMotionProfile.Disable;
		/* When we do start running our state machine start at the beginning. */
		state = 0;
		loopTimeout = -1;
		/*
		 * If application wanted to start an MP before, ignore and wait for next
		 * button press
		 */
		start = false;
	}

	/**
	 * Called every loop.
	 */
	public void control() {
		/* Get the motion profile status every loop */
		leftTalon.getMotionProfileStatus(leftStatus);
		rightTalon.getMotionProfileStatus(rightStatus);

		/*
		 * track time, this is rudimentary but that's okay, we just want to make
		 * sure things never get stuck.
		 */
		if (loopTimeout < 0) {
			/* do nothing, timeout is disabled */
		} else {
			/* our timeout is nonzero */
			if (loopTimeout == 0) {
				/*
				 * something is wrong. Talon is not present, unplugged, breaker
				 * tripped
				 */
				Instrumentation.OnNoProgress();
			} else {
				--loopTimeout;
			}
		}

		/* first check if we are in MP mode */
		if (leftTalon.getControlMode() != ControlMode.MotionProfile
				|| rightTalon.getControlMode() != ControlMode.MotionProfile) {
			/*
			 * we are not in MP mode. We are probably driving the robot around
			 * using gamepads or some other mode.
			 */
			state = 0;
			loopTimeout = -1;
		} else {
			/*
			 * we are in MP control mode. That means: starting Mps, checking Mp
			 * progress, and possibly interrupting MPs if thats what you want to
			 * do.
			 */
			switch (state) {
				case 0: /* wait for application to tell us to start an MP */
					if (start) {
						start = false;
	
						setValue = SetValueMotionProfile.Disable;
						startFilling();
						/*
						 * MP is being sent to CAN bus, wait a small amount of time
						 */
						state = 1;
						loopTimeout = kNumLoopsTimeout;
					}
					break;
				case 1: /*
						 * wait for MP to stream to Talon, really just the first few
						 * points
						 */
					/* do we have a minimum numberof points in Talon */
					if (leftStatus.btmBufferCnt > kMinPointsInTalon
							&& rightStatus.btmBufferCnt > kMinPointsInTalon) {
						/* start (once) the motion profile */
						setValue = SetValueMotionProfile.Enable;
						/* MP will start once the control frame gets scheduled */
						state = 2;
						loopTimeout = kNumLoopsTimeout;
					}
					break;
				case 2: /* check the status of the MP */
					/*
					 * if talon is reporting things are good, keep adding to our
					 * timeout. Really this is so that you can unplug your talon in
					 * the middle of an MP and react to it.
					 */
					if (leftStatus.isUnderrun == false && rightStatus.isUnderrun == false) {
						loopTimeout = kNumLoopsTimeout;
					}
					/*
					 * If we are executing an MP and the MP finished, start loading
					 * another. We will go into hold state so robot servo's
					 * position.
					 */
					if (leftStatus.activePointValid && leftStatus.isLast
							&& rightStatus.activePointValid && rightStatus.isLast) {
						/*
						 * because we set the last point's isLast to true, we will
						 * get here when the MP is done
						 */
						setValue = SetValueMotionProfile.Disable;
						state = 0;
						loopTimeout = -1;
					}
					break;
			}

			/* Get the motion profile status every loop */
			leftTalon.getMotionProfileStatus(leftStatus);
			leftHeading = leftTalon.getActiveTrajectoryHeading();
			leftPos = leftTalon.getActiveTrajectoryPosition();
			leftVel = leftTalon.getActiveTrajectoryVelocity();
			
			rightTalon.getMotionProfileStatus(rightStatus);
			rightHeading = rightTalon.getActiveTrajectoryHeading();
			rightPos = rightTalon.getActiveTrajectoryPosition();
			rightVel = rightTalon.getActiveTrajectoryVelocity();

			/* printfs and/or logging */
			// TODO: Rewrite Instrumentation for both talons? Right now only left.
			//Instrumentation.process(leftStatus, leftPos, leftVel, leftHeading);
		}
	}
	/**
	 * Find enum value if supported.
	 * @param durationMs
	 * @return enum equivalent of durationMs
	 */
	private TrajectoryDuration GetTrajectoryDuration(int durationMs)
	{	 
		/* create return value */
		TrajectoryDuration retval = TrajectoryDuration.Trajectory_Duration_0ms;
		/* convert duration to supported type */
		retval = retval.valueOf(durationMs);
		/* check that it is valid */
		if (retval.value != durationMs) {
			DriverStation.reportError("Trajectory Duration not supported - use configMotionProfileTrajectoryPeriod instead", false);		
		}
		/* pass to caller */
		return retval;
	}
	/** Start filling the MPs to all of the involved Talons. */
	private void startFilling() {
		/* since this example only has one talon, just update that one */
		startFilling(currentTrajectory.getLeftArray(), currentTrajectory.getRightArray(), currentTrajectory.getCount());
	}

	private void startFilling(double[][] leftProfile, double[][] rightProfile, int totalCnt) {

		/* create an empty point */
		TrajectoryPoint leftPoint = new TrajectoryPoint();
		TrajectoryPoint rightPoint = new TrajectoryPoint();

		/* did we get an underrun condition since last time we checked ? */
		if (leftStatus.hasUnderrun) {
			/* better log it so we know about it */
			Instrumentation.OnUnderrun();
			/*
			 * clear the error. This flag does not auto clear, this way 
			 * we never miss logging it.
			 */
			leftTalon.clearMotionProfileHasUnderrun(0);
		}
		/*
		 * just in case we are interrupting another MP and there is still buffer
		 * points in memory, clear it.
		 */
		leftTalon.clearMotionProfileTrajectories();
		rightTalon.clearMotionProfileTrajectories();

		/* set the base trajectory period to zero, use the individual trajectory period below */
		int baseTrajPeriodMs = (int)(leftProfile[0][2] * 1000); // Base the base trajectory period on first point in left profile.
		leftTalon.configMotionProfileTrajectoryPeriod(baseTrajPeriodMs, 0);
		rightTalon.configMotionProfileTrajectoryPeriod(baseTrajPeriodMs, 0);
		
		/* This is fast since it's just into our TOP buffer */
		for (int i = 0; i < totalCnt; ++i) {
			double leftPositionRaw = leftProfile[i][0]; // ft
			double leftVelocityRaw = leftProfile[i][1]; // ft/sec
			/* for each point, fill our structure and pass it to API */
			leftPoint.position = OI.ft2Units(leftPositionRaw);
			leftPoint.velocity = OI.fps2UnitsPerRev(leftVelocityRaw);
			
			// SNEAKY PHOENIX CRAP:
			// This TrajectoryDuration is only allowed to be 0, 5, 10, 20, 30, 40, 50, or 100.
			// Any other value will return 100.
			leftPoint.timeDur = GetTrajectoryDuration(0); // Accounted for using configMotionProfileTrajectory
			
			if (i == 0) {
				leftPoint.zeroPos = true; /* set this to true on the first point */
			} else {
				leftPoint.zeroPos = false;
			}
			if ((i + 1) == totalCnt) {
				leftPoint.isLastPoint = true; /* set this to true on the last point  */
			} else {
				leftPoint.isLastPoint = false;
			}
			
			double rightPositionRaw = rightProfile[i][0]; // ft
			double rightVelocityRaw = rightProfile[i][1]; // ft/sec
			/* for each point, fill our structure and pass it to API */
			
			rightPoint.position = OI.ft2Units(rightPositionRaw);
			rightPoint.velocity = OI.fps2UnitsPerRev(rightVelocityRaw);
			
			// SNEAKY PHOENIX CRAP:
			// This TrajectoryDuration is only allowed to be 0, 5, 10, 20, 30, 40, 50, or 100.
			// Any other value will return 100.
			rightPoint.timeDur = GetTrajectoryDuration(0); // Accounted for using configMotionProfileTrajectoryPeriod
			rightPoint.zeroPos = false;
			if (i == 0)
				rightPoint.zeroPos = true; /* set this to true on the first point */

			rightPoint.isLastPoint = false;
			if ((i + 1) == totalCnt)
				rightPoint.isLastPoint = true; /* set this to true on the last point  */

			if (!currentTrajectory.isReversed()) {
				Robot.driveTrain.invertLeftSide(!RobotMap.LEFT_SIDE_INVERTED);
				Robot.driveTrain.invertRightSide(!RobotMap.RIGHT_SIDE_INVERTED);
				leftTalon.pushMotionProfileTrajectory(leftPoint);
				rightTalon.pushMotionProfileTrajectory(rightPoint);
			} else {
				Robot.driveTrain.invertLeftSide(RobotMap.LEFT_SIDE_INVERTED);
				Robot.driveTrain.invertRightSide(RobotMap.RIGHT_SIDE_INVERTED);
				leftTalon.pushMotionProfileTrajectory(rightPoint);
				rightTalon.pushMotionProfileTrajectory(leftPoint);
			}
		}
	}
	/**
	 * Called by application to signal Talon to start the buffered MP (when it's
	 * able to).
	 */
	public void startMotionProfile() {
		start = true;
	}

	/**
	 * 
	 * @return the output value to pass to Talon's set() routine. 0 for disable
	 *         motion-profile output, 1 for enable motion-profile, 2 for hold
	 *         current motion profile trajectory point.
	 */
	public SetValueMotionProfile getSetValue() {
		return setValue;
	}
}
