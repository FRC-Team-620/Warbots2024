// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jmhsrobotics.frc2024;

import org.jmhsrobotics.frc2024.controlBoard.CompControl;
import org.jmhsrobotics.frc2024.controlBoard.ControlBoard;
import org.jmhsrobotics.frc2024.subsystems.arm.ArmCommand;
import org.jmhsrobotics.frc2024.subsystems.arm.ArmSubsystem;
import org.jmhsrobotics.frc2024.subsystems.drive.DriveConstants;
import org.jmhsrobotics.frc2024.subsystems.drive.DriveSubsystem;
import org.jmhsrobotics.frc2024.subsystems.drive.commands.DriveCommand;
import org.jmhsrobotics.frc2024.subsystems.drive.commands.IntakeCommand;
import org.jmhsrobotics.frc2024.subsystems.vision.VisionSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RobotContainer {

	private ControlBoard control = new CompControl();
	// Subsystems
	private final DriveSubsystem driveSubsystem = new DriveSubsystem();
	private final ArmSubsystem armSubsystem = new ArmSubsystem();

	private final VisionSubsystem vision = new VisionSubsystem(this.driveSubsystem);

	private final SendableChooser<Command> autoChooser;

	public RobotContainer() {

		this.driveSubsystem.setDefaultCommand(new DriveCommand(this.driveSubsystem, this.control));
		SmartDashboard.putData("Schedular", CommandScheduler.getInstance());
		configureBindings();

		armSubsystem.setDefaultCommand(new ArmCommand(0, armSubsystem));

		// Named commands must be added before building the chooser.
		configurePathPlanner();
		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Chooser", autoChooser);
	}

	private void configurePathPlanner() {
		// Add path planner auto chooser.

		AutoBuilder.configureHolonomic(driveSubsystem::getPose, driveSubsystem::resetOdometry,
				driveSubsystem::getChassisSpeeds, driveSubsystem::drive,
				new HolonomicPathFollowerConfig(new PIDConstants(.5, 0, 0), new PIDConstants(1.5, 0, 0),
						DriveConstants.SwerveConstants.kMaxSpeedMetersPerSecond, .5, new ReplanningConfig()),
				this::getAllianceFlipState, driveSubsystem);
		NamedCommands.registerCommand("Intake", new IntakeCommand(driveSubsystem, 5));
		NamedCommands.registerCommand("Wait", new WaitCommand(30));
	}

	// TODO: fix this later to flip correctly based on side color
	// TODO: Check parity of this
	private boolean getAllianceFlipState() {
		return DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == Alliance.Blue : false;
	}

	private void configureBindings() {
	}

	public Command getAutonomousCommand() {
		Command picked = autoChooser.getSelected();
		if (picked == null) {
			return Commands.print("No autonomous command configured");
		} else {
			return picked;
		}
	}

	public DriveSubsystem getDriveSubsystem() {
		return this.driveSubsystem;
	}
}
