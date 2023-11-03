// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.drivetrain.DriveConstants;
import frc.robot.drivetrain.DriveSubsystem;
import frc.robot.drivetrain.DriveConstants.SwerveConstants;

public class DriveCommand extends CommandBase {
  /** Creates a new DriveCommand. */
  private DriveSubsystem driveSubsystem;
  // TODO: Update this control to a control board style input
  private CommandXboxController control;

  public DriveCommand(DriveSubsystem driveSubsystem, CommandXboxController control) {
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
    double xSpeed = MathUtil.applyDeadband(getSquareInput(this.control.getLeftX()) * SwerveConstants.kMaxXSpeed, SwerveConstants.kDeadBand);
    double ySpeed = MathUtil.applyDeadband(getSquareInput(-this.control.getLeftY()) * SwerveConstants.kMaxYSpeed, SwerveConstants.kDeadBand);
    double rotationSpeed = MathUtil.applyDeadband(getSquareInput(this.control.getRightX()) * SwerveConstants.kMaxRotationSpeed, SwerveConstants.kDeadBand);
    SmartDashboard.putNumber("SwerveDriveXSpeed", xSpeed);
    SmartDashboard.putNumber("SwerveDriveYSpeed", ySpeed);
    SmartDashboard.putNumber("SwerveRotationSpeed", rotationSpeed);
    this.driveSubsystem.drive(xSpeed, ySpeed, rotationSpeed, SwerveConstants.kFieldRelative,
        SwerveConstants.kRateLimit);

    // If a/b is pressed on the contorller, set the robot to brake mode/ re-define
    // forward respecitively
    if (this.control.a().getAsBoolean()) {
      this.driveSubsystem.setX();
    } else if (this.control.b().getAsBoolean()) {
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
    return Math.pow(input, 2)*Math.signum(input);
  }

}
