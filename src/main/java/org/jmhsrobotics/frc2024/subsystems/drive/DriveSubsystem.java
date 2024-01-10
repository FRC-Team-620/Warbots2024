// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jmhsrobotics.frc2024.subsystems.drive;

import org.jmhsrobotics.frc2024.subsystems.drive.DriveConstants.AutoConstants;
import org.jmhsrobotics.frc2024.subsystems.drive.DriveConstants.SwerveConstants;
import org.jmhsrobotics.frc2024.subsystems.drive.swerve.ISwerveModule;
import org.jmhsrobotics.frc2024.subsystems.drive.swerve.MAXSwerveModule;
import org.jmhsrobotics.frc2024.subsystems.drive.swerve.RevSwerveDrive;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private ISwerveModule m_frontLeft = new MAXSwerveModule(
      SwerveConstants.kFrontLeftDrivingCanId,
      SwerveConstants.kFrontLeftTurningCanId,
      SwerveConstants.kFrontLeftChassisAngularOffset);

  private ISwerveModule m_frontRight = new MAXSwerveModule(
      SwerveConstants.kFrontRightDrivingCanId,
      SwerveConstants.kFrontRightTurningCanId,
      SwerveConstants.kFrontRightChassisAngularOffset);

  private ISwerveModule m_rearLeft = new MAXSwerveModule(
      SwerveConstants.kRearLeftDrivingCanId,
      SwerveConstants.kRearLeftTurningCanId,
      SwerveConstants.kBackLeftChassisAngularOffset);

  private ISwerveModule m_rearRight = new MAXSwerveModule(
      SwerveConstants.kRearRightDrivingCanId,
      SwerveConstants.kRearRightTurningCanId,
      SwerveConstants.kBackRightChassisAngularOffset);

  // Create gyro
  private final Pigeon2 m_gyro = new Pigeon2(SwerveConstants.kGyroCanId);

  // Create RevSwerveDrive
  private final RevSwerveDrive swerveDrive = new RevSwerveDrive(m_frontLeft, m_frontRight, m_rearLeft, m_rearRight,
      m_gyro);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    AutoBuilder.configureHolonomic(swerveDrive::getPose, swerveDrive::resetOdometry, swerveDrive::getChassisSpeeds,
        this::drive,
        new HolonomicPathFollowerConfig(
            new PIDConstants(
                AutoConstants.kPathTranslationP,
                AutoConstants.kPathTranslationI,
                AutoConstants.kPathTranslationD),
            new PIDConstants(
                AutoConstants.kPathRotationP,
                AutoConstants.kPathRotationI,
                AutoConstants.kPathRotationD),
            SwerveConstants.kMaxSpeedMetersPerSecond,
            SwerveConstants.kTrackWidth / 2,
            new ReplanningConfig()),
        this::getAllianceFlipState,
        this);

  }

  // TODO: fix this later to flip correctly based on side color
  private boolean getAllianceFlipState() {
    return DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == Alliance.Blue
        : false;
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
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
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
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, SwerveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    // if sideways became for/backward somehow, change it to 0 degree
    m_gyro.setYaw(90);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  // public double getTurnRate() {
  // double[] vels = new double[3];
  // m_gyro.getRawGyro(vels);
  // m_gyro
  // return vels[2] * (SwerveConstants.kGyroReversed ? -1.0 : 1.0);
  // }
  // yee haw brother

  // stop driving(for future commands and driveCommand)
  public void stopDrive() {
    this.drive(0, 0, 0, SwerveConstants.kFieldRelative, SwerveConstants.kRateLimit);
  }

  private final Pigeon2SimState imuSim = m_gyro.getSimState();

  @Override
  public void simulationPeriodic() {
    m_frontLeft.update(0.02);
    m_frontRight.update(0.02);
    m_rearLeft.update(0.02);
    m_rearRight.update(0.02);
    Twist2d twist = SwerveConstants.kDriveKinematics.toTwist2d(new SwerveModulePosition[] { m_frontLeft.getPosition(),
        m_frontRight.getPosition(), m_rearLeft.getPosition(), m_rearRight.getPosition() });
    imuSim.addYaw(twist.dtheta); // TODO: this may need to be converted back somthing is wrong elsewhere.
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
