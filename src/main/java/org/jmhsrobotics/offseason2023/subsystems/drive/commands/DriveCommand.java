// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jmhsrobotics.offseason2023.subsystems.drive.commands;

import org.jmhsrobotics.offseason2023.controlBoard.ControlBoard;
import org.jmhsrobotics.offseason2023.subsystems.drive.DriveConstants;
import org.jmhsrobotics.offseason2023.subsystems.drive.DriveSubsystem;
import org.jmhsrobotics.offseason2023.subsystems.drive.DriveConstants.SwerveConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DriveCommand extends CommandBase {
  /** Creates a new DriveCommand. */
  private DriveSubsystem driveSubsystem;
  // TODO: Update this control to a control board style input
  private ControlBoard control;

  public DriveCommand(DriveSubsystem driveSubsystem, ControlBoard control) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.driveSubsystem = driveSubsystem;
    this.control = control;

    addRequirements(this.driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // stop drive is a method to set speed inputs to zero and angle the wheels in
    // brake angle
    this.driveSubsystem.stopDrive();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //
    double xSpeed = MathUtil.applyDeadband(this.getSquareInput(this.control.xInput()) * SwerveConstants.kMaxXSpeed,
        SwerveConstants.kDeadBand);
    double ySpeed = MathUtil.applyDeadband(this.getSquareInput(-this.control.yInput()) * SwerveConstants.kMaxYSpeed,
        SwerveConstants.kDeadBand);
    double rotationSpeed = MathUtil.applyDeadband(
        this.getSquareInput(this.control.rotationalInput()) * SwerveConstants.kMaxRotationSpeed,
        SwerveConstants.kDeadBand);
    SmartDashboard.putNumber("SwerveDrive/Input/SwerveDriveXSpeed", xSpeed);
    SmartDashboard.putNumber("SwerveDrive/Input/SwerveDriveXSpeed", ySpeed);
    SmartDashboard.putNumber("SwerveDrive/Input/SwerveDriveXSpeed", rotationSpeed);
    this.driveSubsystem.drive(xSpeed, ySpeed, rotationSpeed, SwerveConstants.kFieldRelative,
        SwerveConstants.kRateLimit);

    // If a/b is pressed on the contorller, set the robot to brake mode/ re-define
    // forward respecitively

    // control.brake() returns a trigger value(not a boolean), so use .getAsBoolean
    // to convert it to a boolean(same for setZeroHeading)
    if (this.control.brake().getAsBoolean()) {
      this.driveSubsystem.setX();
    } else if (this.control.setZeroHeading().getAsBoolean()) {
      // zero heading is a method in driveSubsystem that "redefine" forward for the robot
      this.driveSubsystem.zeroHeading();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.driveSubsystem.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // This method square the input(for less sensitive control)
  private double getSquareInput(double input) {
    return Math.pow(input, 2) * Math.signum(input);
  }

}
