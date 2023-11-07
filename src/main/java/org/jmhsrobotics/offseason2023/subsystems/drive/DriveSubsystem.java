// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jmhsrobotics.offseason2023.subsystems.drive;

import org.jmhsrobotics.offseason2023.Robot;
import org.jmhsrobotics.offseason2023.subsystems.drive.DriveConstants.SwerveConstants;
import org.jmhsrobotics.offseason2023.utils.SwerveUtils;
import org.jmhsrobotics.warcore.swerve.SwerveVisualizer;

import com.ctre.phoenix.platform.DeviceType;
import com.ctre.phoenix.platform.PlatformJNI;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Rev Code Dont Touch!
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

  // The gyro sensor
  // private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  private final Pigeon2 m_gyro = new Pigeon2(30);

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(SwerveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(SwerveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      SwerveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getYaw()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

    SwerveVisualizer visualizer;
    double simRot = 0;
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    visualizer = new SwerveVisualizer(1, 1);
    if (Robot.isSimulation()) {
      m_frontLeft = new SimSwerveModule();
      m_frontRight = new SimSwerveModule();
      m_rearLeft = new SimSwerveModule();
      m_rearRight = new SimSwerveModule();
    }
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getYaw()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getYaw()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
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

    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(SwerveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        } else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      } else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * SwerveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * SwerveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * SwerveConstants.kMaxAngularSpeed;

    var swerveModuleStates = SwerveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(m_gyro.getYaw()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, SwerveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
    simRot +=  rotDelivered; //TODO: Wrong for sim
    visualizer.update(m_frontLeft.getState().angle, m_frontRight.getState().angle, m_rearLeft.getState().angle, m_rearRight.getState().angle, getPose());
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
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
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getYaw()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    double[] vels = new double[3];
    m_gyro.getRawGyro(vels);
    return vels[2] * (SwerveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  // stop driving(for future commands and driveCommand)
  public void stopDrive() {
    this.setX();
    this.drive(0, 0, 0, SwerveConstants.kFieldRelative, SwerveConstants.kRateLimit);
  }

  @Override
  public void simulationPeriodic() {
    m_frontLeft.update(0.02);
    m_frontRight.update(0.02);
    m_rearLeft.update(0.02);
    m_rearRight.update(0.02);
    PlatformJNI.JNI_SimSetPhysicsInput(DeviceType.PigeonIMU.value, 30, "HeadingRaw",
				-MathUtil.inputModulus(simRot, -180, 180));
  }
}
