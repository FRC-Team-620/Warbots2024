// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.controlBoard.CompControl;
import frc.robot.controlBoard.ControlBoard;
import frc.robot.drivetrain.DriveSubsystem;
import frc.robot.drivetrain.commands.DriveCommand;

public class RobotContainer {

  private ControlBoard control = new CompControl();
  private DriveSubsystem driveSubsystem = new DriveSubsystem();

  public RobotContainer() {

    this.driveSubsystem.setDefaultCommand(new DriveCommand(this.driveSubsystem, this.control));
    
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public DriveSubsystem getDriveSubsystem(){
    return this.driveSubsystem;
  }
}
