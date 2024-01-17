// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jmhsrobotics.frc2024.subsystems.drive;

import org.jmhsrobotics.frc2024.Robot;
import org.jmhsrobotics.frc2024.subsystems.drive.DriveConstants.SwerveConstants;
import org.jmhsrobotics.frc2024.subsystems.drive.swerve.ISwerveModule;
import org.jmhsrobotics.frc2024.subsystems.drive.swerve.MAXSwerveModule;
import org.jmhsrobotics.frc2024.subsystems.drive.swerve.RevSwerveDrive;
import org.jmhsrobotics.frc2024.subsystems.drive.swerve.SimSwerveModule;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
	// Create MAXSwerveModules
	//Test
	private ISwerveModule m_frontLeft;

	private ISwerveModule m_frontRight;

	private ISwerveModule m_rearLeft;

	private ISwerveModule m_rearRight;

	// Create gyro
	private final Pigeon2 m_gyro = new Pigeon2(SwerveConstants.kGyroCanId);

	// Create RevSwerveDrive
	private final RevSwerveDrive swerveDrive;

	/** Creates a new DriveSubsystem. */
	public DriveSubsystem() {
		if (Robot.isSimulation()) {
			//test
			m_frontLeft = new SimSwerveModule();
			m_frontRight = new SimSwerveModule();
			m_rearLeft = new SimSwerveModule();
			m_rearRight = new SimSwerveModule();
		} else {
			m_frontLeft = new MAXSwerveModule(SwerveConstants.kFrontLeftDrivingCanId,
					SwerveConstants.kFrontLeftTurningCanId, SwerveConstants.kFrontLeftChassisAngularOffset);

			m_frontRight = new MAXSwerveModule(SwerveConstants.kFrontRightDrivingCanId,
					SwerveConstants.kFrontRightTurningCanId, SwerveConstants.kFrontRightChassisAngularOffset);

			m_rearLeft = new MAXSwerveModule(SwerveConstants.kRearLeftDrivingCanId,
					SwerveConstants.kRearLeftTurningCanId, SwerveConstants.kBackLeftChassisAngularOffset);

			m_rearRight = new MAXSwerveModule(SwerveConstants.kRearRightDrivingCanId,
					SwerveConstants.kRearRightTurningCanId, SwerveConstants.kBackRightChassisAngularOffset);
		}

		swerveDrive = new RevSwerveDrive(m_frontLeft, m_frontRight, m_rearLeft, m_rearRight, m_gyro);
	}

	@Override
	public void periodic() {
		// Update the odometry in the periodic block
		swerveDrive.updateOdometry();
	}

	public void drive(ChassisSpeeds chassisSpeeds) {
		double xSpeed = chassisSpeeds.vxMetersPerSecond / SwerveConstants.kMaxSpeedMetersPerSecond;
		double ySpeed = chassisSpeeds.vyMetersPerSecond / SwerveConstants.kMaxSpeedMetersPerSecond;
		double rotation = chassisSpeeds.omegaRadiansPerSecond / SwerveConstants.kMaxAngularSpeed;
		this.drive(xSpeed, ySpeed, rotation, false, false);
	}

	/**
	 * Method to drive the robot using joystick info.
	 *
	 * @param xSpeed
	 *            Speed of the robot in the x direction (forward).
	 * @param ySpeed
	 *            Speed of the robot in the y direction (sideways).
	 * @param rot
	 *            Angular rate of the robot.
	 * @param fieldRelative
	 *            Whether the provided x and y speeds are relative to the field.
	 * @param rateLimit
	 *            Whether to enable rate limiting for smoother control.
	 */
	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
		swerveDrive.drive(xSpeed, ySpeed, rot, fieldRelative, rateLimit);
	}

	public void brake() {
		swerveDrive.setX();
	}

	/**
	 * Sets the swerve ModuleStates.
	 *
	 * @param desiredStates
	 *            The desired SwerveModule states.
	 */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.kMaxSpeedMetersPerSecond);
		m_frontLeft.setDesiredState(desiredStates[0]);
		m_frontRight.setDesiredState(desiredStates[1]);
		m_rearLeft.setDesiredState(desiredStates[2]);
		m_rearRight.setDesiredState(desiredStates[3]);
	}

	/** Resets the drive encoders to currently read a position of 0. */
	public void resetEncoders() {
		m_frontLeft.resetEncoders();
		m_frontRight.resetEncoders();
		m_rearLeft.resetEncoders();
		m_rearRight.resetEncoders();
	}

	/** Zeroes the heading of the robot. */
	public void zeroHeading() {
		// if sideways became for/backward somehow, change it to 0 degree
		m_gyro.setYaw(0);
	}

	/**
	 * Returns the turn rate of the robot.
	 *
	 * @return The turn rate of the robot, in degrees per second
	 */
	// public double getTurnRate() {
	// double[] vels = new double[3];
	// m_gyro.getRawGyro(vels);
	// return vels[2] * (SwerveConstants.kGyroReversed ? -1.0 : 1.0);
	// }
	// yee haw brother

	// stop driving(for future commands and driveCommand)
	public void stopDrive() {
		this.drive(0, 0, 0, SwerveConstants.kFieldRelative, SwerveConstants.kRateLimit);
	}

	private final Pigeon2SimState imuSim = m_gyro.getSimState();
	public Pose2d simpos = new Pose2d();
	@Override
	public void simulationPeriodic() {
		var prevPos = new SwerveDriveWheelPositions(new SwerveModulePosition[]{m_frontLeft.getPosition(),
				m_frontRight.getPosition(), m_rearLeft.getPosition(), m_rearRight.getPosition()}).copy();
		m_frontLeft.update(0.2);
		m_frontRight.update(0.2);
		m_rearLeft.update(0.2);
		m_rearRight.update(0.2);
		var currpos = new SwerveDriveWheelPositions(new SwerveModulePosition[]{m_frontLeft.getPosition(),
				m_frontRight.getPosition(), m_rearLeft.getPosition(), m_rearRight.getPosition()}).copy();
		var twist = SwerveConstants.kDriveKinematics.toTwist2d(prevPos, currpos);
		simpos = getPose().exp(twist);
		imuSim.addYaw(Math.toDegrees(twist.dtheta));
	}

	public Pose2d getPose() {
		return swerveDrive.getPose();
	}

	public void resetOdometry(Pose2d pose2d) {
		swerveDrive.resetOdometry(pose2d);
	}

	public ChassisSpeeds getChassisSpeeds() {
		return swerveDrive.getChassisSpeeds();
	}
}
