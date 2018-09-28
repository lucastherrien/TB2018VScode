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

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

import javax.management.DescriptorRead;

import org.usfirst.frc.team4561.robot.Robot;
import org.usfirst.frc.team4561.robot.RobotMap;

/**
 * Based on https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java/MotionProfile/src/org/usfirst/frc/team217/robot/MotionProfileExample.java
 * @author Kaiz
 *
 */
public class MotionProfileOnboardRunner {
	
	public static Path leftScaleLeft = new LeftScaleLeft();
	public static Path leftScaleRight = new LeftScaleRight();
	public static Path leftScaleTurnAroundS3 = new LeftScaleTurnAroundS3();
	public static Path midSwitchLeft = new MidSwitchLeft();
	public static Path midSwitchRight = new MidSwitchRight();
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
	public static Path scaleLeftCubeLeft = new ScaleLeftCubeLeft();
	public static Path cubeLeftScaleLeft = new CubeLeftScaleLeft();
	public static Path scaleRightCubeRight = new ScaleRightCubeRight();
	public static Path cubeRightScaleRight = new CubeRightScaleRight();
	public static Path scaleLeftCubeLeft2 = new ScaleLeftCubeLeft2();
	public static Path rightScaleLeftZoom = new RightScaleLeftZoom();
	
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
		TestTrajectory(testTrajectory),
		ScaleLeftCubeLeft(scaleLeftCubeLeft),
		CubeLeftScaleLeft(cubeLeftScaleLeft),
		ScaleRightCubeRight(scaleRightCubeRight),
		CubeRightScaleRight(cubeRightScaleRight),
		ScaleLeftCubeLeft2(scaleLeftCubeLeft2),
		RightScaleLeftZoom(rightScaleLeftZoom);
		
		Path trajectory;
		TrajectorySelect(Path trajectory) {
			this.trajectory = trajectory;
		}
		/**
		 * @return The trajectory for left side of the drivetrain.
		 */
		Trajectory getLeftTrajectory() {
			return trajectory.getLeftTrajectory();
		}
		/**
		 * @return The trajectory for right side of the drivetrain.
		 */
		Trajectory getRightTrajectory() {
			return trajectory.getRightTrajectory();
		}
		/**
		 * @return An array of waypoints for the left side of the trajectory, each waypoint containing position, velocity, and time step. 
		 */
		@SuppressWarnings("unused")
		double[][] getLeftArray() {
			return trajectory.getLeftArray();
		}
		/**
		 * @return An array of waypoints for the right side of the trajectory, each waypoint containing position, velocity, and time step. 
		 */
		@SuppressWarnings("unused")
		double[][] getRightArray() {
			return trajectory.getRightArray();
		}
		/**
		 * @return The number of waypoints in the trajectory. This will be the length of the left and right arrays.
		 */
		@SuppressWarnings("unused") int getCount() {
			return trajectory.getCount();
		}
		/**
		 * @return Whether or not the robot goes backwards for this trajectory.
		 */
		@SuppressWarnings("unused") boolean isReversed() {
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

	/** Additional cache for holding the active trajectory points */
	double leftPos = 0, leftVel = 0, leftHeading = 0, rightPos = 0, rightVel = 0, rightHeading = 0;

	/**
	 * Create a notifier to ensure that our method for setting motor speeds is being called
	 * at a constant, reliable rate.
	 */
	class PeriodicRunnable implements java.lang.Runnable {
	    public void run() {
	    	control();
	    }
	}
	Notifier notifier = new Notifier(new PeriodicRunnable());
	
	/**
	 * Reference to the talons we plan on manipulating. We will not changeMode()
	 * or call set(), just get motion profile status and make decisions based on
	 * motion profile.
	 */
	private TalonSRX leftTalon;
	private TalonSRX rightTalon;
	
	/**
	 * Encoder followers to calculate the voltages to set our talons to.
	 */
	EncoderFollower leftFollower = new EncoderFollower();
	EncoderFollower rightFollower = new EncoderFollower();
	
	private final double kP = 0.25; // Proportional gain: 0.25 for Kongo
	private final double kI = 0; // Integral gain (UNIMPLEMENTED in EncoderFollower)
	private final double kD = 0.1; // Derivative gain
	private final double kV = 1.0 / RobotMap.MAX_FEET_PER_SECOND;
	private final double kA = 0; // Should be able to be left at 0 if max accel is configured correctly in trajectory config
	private final double kG = 0;//.075;//.075; //0.075; // Gyro gain: 0.075 for Kongo
	private double startAngle = 0;
	
	/**
	 * If start() gets called, this flag is set and in the control() we will
	 * service it.
	 */
	private boolean start = false;
	
	/**
	 * @param leftTalon reference to left master TalonSRX object.
	 * @param rightTalon reference to the right master TalonSRX object.
	 * @param leftArray array containing the left motion profile
	 * @param rightArray array containing the right motion profile
	 */
	public MotionProfileOnboardRunner(TalonSRX leftTalon, TalonSRX rightTalon) {
		this.leftTalon = leftTalon;
		this.rightTalon = rightTalon;
		prepare();
		leftFollower.configureEncoder((int) (Robot.driveTrain.getLeftPos() * RobotMap.ONBOARD_ENCODER_MULTIPLIER), RobotMap.UNITS_PER_REVOLUTION, RobotMap.WHEEL_DIAMETER / 12);
		rightFollower.configureEncoder((int) (Robot.driveTrain.getRightPos() * RobotMap.ONBOARD_ENCODER_MULTIPLIER), RobotMap.UNITS_PER_REVOLUTION, RobotMap.WHEEL_DIAMETER / 12);
		leftFollower.configurePIDVA(kP, kI, kD, kV, kA);
		rightFollower.configurePIDVA(kP, kI, kD, kV, kA);
		notifier.startSingle(RobotMap.TIME_STEP); // TODO: Try turning up the time step and see how things change
	}
	
	public void prepare() {
		Robot.driveTrain.invertLeftSide(!RobotMap.LEFT_SIDE_INVERTED);
		Robot.driveTrain.invertRightSide(!RobotMap.RIGHT_SIDE_INVERTED);
		Robot.driveTrain.setSensorPhase(RobotMap.LEFT_SIDE_SENSOR_PHASE_REVERSED, RobotMap.RIGHT_SIDE_SENSOR_PHASE_REVERSED);
		Robot.driveTrain.resetEncoders();
		Robot.gyro.reset();
		if (getCurrentTrajectory() != null)
			startAngle = getCurrentTrajectory().trajectory.trajectory.segments[0].heading - Robot.gyro.getAngle();
		else
			startAngle = 0;
		System.out.println("***"+startAngle+"***"
				+ "");
	}
	
	private double normalizeAngle(double angle) {
		double norm = angle % 360.0;
		if (Math.abs(norm) > 180.0) {
			norm = (norm < 0) ? norm + 360.0 : norm - 360.0;
		}
		return norm;
	}
	
	/**
	 * Called to clear Motion profile buffer and reset state info during
	 * disabled and when Talon is not in MP control mode.
	 */
	public void reset() {
		leftFollower.configureEncoder((int) (Robot.driveTrain.getLeftPos() * RobotMap.ONBOARD_ENCODER_MULTIPLIER), RobotMap.UNITS_PER_REVOLUTION, RobotMap.WHEEL_DIAMETER / 12);
		rightFollower.configureEncoder((int) (Robot.driveTrain.getRightPos() * RobotMap.ONBOARD_ENCODER_MULTIPLIER), RobotMap.UNITS_PER_REVOLUTION, RobotMap.WHEEL_DIAMETER / 12);
		Robot.driveTrain.invertLeftSide(RobotMap.LEFT_SIDE_INVERTED);
		Robot.driveTrain.invertRightSide(RobotMap.RIGHT_SIDE_INVERTED);
		start = false;
		startAngle = 0;
	}

	volatile int segment = 0;
	
	/**
	 * Must be called every loop.
	 */
	public void control() {
		double highestOutput = 1;
		if (start) {
			segment++;
			double leftOutputRaw;
			double rightOutputRaw;
			if (!getCurrentTrajectory().isReversed()) {
				leftOutputRaw = leftFollower.calculate((int) (Robot.driveTrain.getLeftPos() * RobotMap.ONBOARD_ENCODER_MULTIPLIER));
				rightOutputRaw = rightFollower.calculate((int) (Robot.driveTrain.getRightPos() * RobotMap.ONBOARD_ENCODER_MULTIPLIER));
			}
			else {
				leftOutputRaw = leftFollower.calculate((int) (-Robot.driveTrain.getLeftPos() * RobotMap.ONBOARD_ENCODER_MULTIPLIER));
				rightOutputRaw = rightFollower.calculate((int) (-Robot.driveTrain.getRightPos() * RobotMap.ONBOARD_ENCODER_MULTIPLIER));
			}
			
			
			double angleDifference;
			double turn;
			if (getCurrentTrajectory().isReversed()) {
				double gyro_heading = normalizeAngle(Robot.gyro.getYaw() + startAngle); // Get our angle in degrees from -180..180
				// Add 180 to desired_heading because bot's reversed
				double desired_heading = normalizeAngle(Pathfinder.boundHalfDegrees(Pathfinder.r2d(leftFollower.getHeading()) + 180));
				angleDifference = normalizeAngle(Pathfinder.boundHalfDegrees(desired_heading + gyro_heading));
				turn = kG * (1.0/80.0) * angleDifference;
			} else {
				double gyro_heading = normalizeAngle(Robot.gyro.getYaw() + startAngle); // Get our angle in degrees from -180..180
				double desired_heading = normalizeAngle(Pathfinder.r2d(leftFollower.getHeading()));
				angleDifference = normalizeAngle(Pathfinder.boundHalfDegrees(desired_heading + gyro_heading));
				SmartDashboard.putNumber("Desired Angle", desired_heading);
				turn = kG * (-1.0/80.0) * angleDifference;
			}
			
			SmartDashboard.putNumber("angle error", angleDifference);
			SmartDashboard.putNumber("turn", turn);
			double leftOutput = leftOutputRaw + turn;
			double rightOutput = rightOutputRaw - turn;
			
			if (Math.abs(leftOutput) > 1 || Math.abs(rightOutput) > 1) {
				if (Math.abs(leftOutput) > Math.abs(rightOutput)) {
					highestOutput = Math.abs(leftOutput);
					leftOutput = leftOutput/leftOutput;
					rightOutput = rightOutput/leftOutput;
				}
				else {
					highestOutput = Math.abs(rightOutput);
					leftOutput = leftOutput/rightOutput;
					rightOutput = rightOutput/rightOutput;
				}
			}
			SmartDashboard.putNumber("left output" , leftOutput);
			SmartDashboard.putNumber("right output" , rightOutput);
			if(getCurrentTrajectory().isReversed()) {
				leftTalon.set(ControlMode.PercentOutput, -rightOutput);
				rightTalon.set(ControlMode.PercentOutput, -leftOutput);
			} else {
				leftTalon.set(ControlMode.PercentOutput, leftOutput);
				rightTalon.set(ControlMode.PercentOutput, rightOutput);
			}
			/* Get the motion profile status every loop */
			//leftHeading = leftFollower.getHeading();
			//leftPos = leftFollower.getSegment().position;
			//leftVel = leftFollower.getSegment().velocity;
			
			//rightHeading = rightFollower.getHeading();
			//rightPos = rightFollower.getSegment().position;
			//rightVel = rightFollower.getSegment().velocity;
			
			if (leftFollower.isFinished() && rightFollower.isFinished()) {
				System.out.println("Trajectory complete.");
				Robot.driveTrain.stop();
				start = false;
				segment = 0;
				reset();
			}
		}
		// TODO: Instrumentation implementation for onboard
		notifier.startSingle(RobotMap.TIME_STEP * highestOutput);
	}

	/**
	 * Call to start running the current trajectory.
	 */
	public void startMotionProfile() {
		Robot.driveTrain.resetEncoders();
		leftFollower.setTrajectory(getCurrentTrajectory().getLeftTrajectory());
		rightFollower.setTrajectory(getCurrentTrajectory().getRightTrajectory());
		leftFollower.configureEncoder((int) (Robot.driveTrain.getLeftPos() * RobotMap.ONBOARD_ENCODER_MULTIPLIER), RobotMap.UNITS_PER_REVOLUTION, RobotMap.WHEEL_DIAMETER / 12);
		rightFollower.configureEncoder((int) (Robot.driveTrain.getRightPos() * RobotMap.ONBOARD_ENCODER_MULTIPLIER), RobotMap.UNITS_PER_REVOLUTION, RobotMap.WHEEL_DIAMETER / 12);
		start = true;
	}
	
	public boolean isFinished() {
		return leftFollower.isFinished() && rightFollower.isFinished();
	}
	
	/**
	 * @return time elapsed in seconds since the current trajectory started. Returns -1 if current trajectory is not running.
	 */
	public double getTimeElapsed() {
		int index = getCurrentSegmentIndex();
		if (index == -1) { return -1; }
		double timePassed = getCurrentSegmentIndex() * currentTrajectory.getLeftTrajectory().get(0).dt;
		return timePassed;
	}
	
	/**
	 * @return time in seconds until the current trajectory ends. Returns -1 if current trajectory is not running.
	 */
	public double getTimeRemaining() {
		int index = getCurrentSegmentIndex();
		if (index == -1) { return -1; }
		double timePassed = getCurrentSegmentIndex() * currentTrajectory.getLeftTrajectory().get(0).dt;
		return getTotalTime() - timePassed;
	}
	
	/**
	 * @return time in seconds that running the full current trajectory will take.
	 */
	public double getTotalTime() {
		return currentTrajectory.getCount() * currentTrajectory.getLeftTrajectory().get(0).dt;
	}
	
	/**
	 * @return the 0-based index of the segment currently being run. Returns -1 if a trajectory is 
	 * not being run right now or if an ArrayIndexOutOfBoundsException occurs.
	 */
	public int getCurrentSegmentIndex() {
		int index = -1;
		if (start) {
			index = segment;
		}
		return index;
	}
}
