package org.usfirst.frc.team4561.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Gyroscope extends Subsystem {
	
	private AHRS gyro;
	private ADXRS450_Gyro backupGyro;
	private boolean primary = true;
	private boolean real = true;
	
	public Gyroscope() {
		try {
			gyro = new AHRS(SPI.Port.kMXP);
		}
		catch (RuntimeException ex) {
			DriverStation.reportError("Failed to create Gyro - is it plugged in?", false);
			System.out.println("Falling back to backup gyro");
			primary = false;
			try {
				backupGyro = new ADXRS450_Gyro();
			}
			catch (RuntimeException e) {
				DriverStation.reportError("Failed to create backup Gyro - is it plugged in?", false);
				real = false;
			}
		}
		if (getUpdate() == 0) {
			primary = false;
			DriverStation.reportError("Failed to connect to Gyro - is it plugged in?", false);
			try {
				backupGyro = new ADXRS450_Gyro();
			}
			catch (RuntimeException ex) {
				real = false;
				DriverStation.reportError("Failed to connect to backup Gyro - is it plugged in?", false);
			}
		}
		}
	public void checkGyro() {
		if (getUpdate() == 0 && primary) {
			primary = false;
			//DriverStation.reportError("Failed to connect to Gyro - is it plugged in?", false);
			try {
				backupGyro = new ADXRS450_Gyro();
			}
			catch (RuntimeException ex) {
				real = false;
				//DriverStation.reportError("Failed to connect to backup Gyro - is it plugged in?", false);
			}
		}
		else if (getUpdate() != 0 && !primary) {
			primary = true;
			System.out.println("Main Gyro reconnected");
		}
	}
	public int getUpdate() {
		if (primary) {
			try {
				return gyro.getActualUpdateRate();
			}
			catch (RuntimeException e) {
				return 0;
			}
		}
		else {
			return 0;
		}
	}
	public double getPitch() {
		if (primary) return gyro.getPitch();
		else return 0;
	}
	public double getRoll() {
		if (primary) return gyro.getRoll();
		else return 0;
	}
	public double getYaw() {
		if (primary) return gyro.getYaw();
		else if (real) return backupGyro.getAngle();
		else return 0;
	}
	public double getAngle() {
		if (primary) return gyro.getAngle();
		else if (real) return backupGyro.getAngle();
		else return 0;
	}
	public double getRate() {
		if (primary) return gyro.getRate();
		else if (real) return backupGyro.getRate();
		else return 0;
	}
	public boolean isPrimary() {
		return primary;
	}
	public double getHead() {
		if (primary) return gyro.getCompassHeading();
		else return 0;
	}
	public double getFusedHeading() {
		if (primary) return gyro.getFusedHeading();
		else return 0;
	}
	public void reset() {
		if (primary) gyro.reset();
	}
	public double getPressure() {
		if (primary) return gyro.getBarometricPressure();
		else return 0;
	}
	public void set(double angle) {
		gyro.setAngleAdjustment(angle);
	}
	public void unset() {
		gyro.setAngleAdjustment(0);
	}
	public double getAltitude() {
		if (primary) return gyro.getAltitude();
		else return 0;
	}
	public double getAccel() {
		if (primary) return gyro.getAccelFullScaleRangeG();
		else return 0;
	}
	public double getTemp() {
		if (primary) return gyro.getTempC();
		else return 0;
	}
	public double getSpeedX() {
		if (primary) return gyro.getVelocityX();
		else return 0;
	}
	public double getSpeedY() {
		if (primary) return gyro.getVelocityY();
		else return 0;
	}
	public double getSpeedZ() {
		if (primary) return gyro.getVelocityZ();
		else return 0;
	}
	public double getDisplacementX() {
		if (primary) return gyro.getDisplacementX();
		else return 0;
	}
	public double getDisplacementY() {
		if (primary) return gyro.getDisplacementY();
		else return 0;
	}
	public double getDisplacementZ() {
		if (primary) return gyro.getDisplacementZ();
		else return 0;
	}
	public boolean isTipping() { //TODO: needs tuning probably
		return (getPitch() > 30 && getPitch() < 80) || (getRoll() > 25 && getRoll() < 80);
	}
	public boolean isTipped() {
		return (getPitch() >= 80) || (getRoll() >= 80);
	}
	public boolean isMoving() {
		if (primary) return gyro.isMoving();
		else return false;
	}
	public boolean isRotating() {
		if (primary) return gyro.isRotating();
		else return false;
	}
	public boolean isReal() {
		return real;
	}
	public boolean isDisturbed() {
		if (primary) return gyro.isMagneticDisturbance();
		else return false;
	}
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub

	}

}
