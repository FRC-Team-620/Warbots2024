// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jmhsrobotics.frc2024;

import org.jmhsrobotics.frc2024.controlBoard.CompControl;
import org.jmhsrobotics.frc2024.controlBoard.ControlBoard;
import org.jmhsrobotics.frc2024.subsystems.drive.DriveSubsystem;
import org.jmhsrobotics.frc2024.subsystems.drive.commands.DriveCommand;
import org.jmhsrobotics.frc2024.subsystems.drive.commands.auto.DriveTimeCommand;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import monologue.Logged;

public class RobotContainer implements Logged {

	public ControlBoard control = new CompControl();
	// Subsystems
	private final DriveSubsystem driveSubsystem = new DriveSubsystem();

	// private final VisionSubsystem visionSubsystem = new
	// VisionSubsystem(this.driveSubsystem);

	// private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
	// private final LEDSubsystem ledSubsystem = new LEDSubsystem();
	// private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

	// private final ArmSubsystem armSubsystem = new ArmSubsystem();

	private final SendableChooser<Command> autoChooser;

	public RobotContainer() {

		this.driveSubsystem.setDefaultCommand(new DriveCommand(this.driveSubsystem, this.control));
		// this.ledSubsystem.setDefaultCommand(new
		// RainbowLEDCommand(this.ledSubsystem));

		SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
		// SmartDashboard.putData("LockAprilTagCommand", new LockAprilTag(7,
		// this.driveSubsystem, this.visionSubsystem));
		// SmartDashboard.putData("ArmCommand", new ArmCommand(0, this.armSubsystem));
		configureBindings();

		// Named commands must be added before building the chooser.
		configurePathPlanner();
		autoChooser = AutoBuilder.buildAutoChooser();
		autoChooser.setDefaultOption("BaseLineAuto", new DriveTimeCommand(1.535, 0.3, driveSubsystem));
		SmartDashboard.putData("Auto Chooser", autoChooser);
		// ShooterCommand shooterCommand = new ShooterCommand(2000, shooterSubsystem);
		// SmartDashboard.putData("Shooter Command", shooterCommand);
	}

	private void configurePathPlanner() {
		// Add path planner auto chooser.

		AutoBuilder.configureHolonomic(driveSubsystem::getPose, driveSubsystem::resetOdometry,
				driveSubsystem::getChassisSpeeds, driveSubsystem::drive,
				new HolonomicPathFollowerConfig(new PIDConstants(.5, 0, 0), new PIDConstants(1.5, 0, 0),
						Constants.SwerveConstants.kMaxSpeedMetersPerSecond, .5, new ReplanningConfig()),
				this::getAllianceFlipState, driveSubsystem);

		// TODO: fix command names in pathplanner and code

		// NamedCommands.registerCommand("ScoreAmp", new ArmCommand(86,
		// this.armSubsystem));
		// NamedCommands.registerCommand("Extake", new
		// ExtakeCommand(this.intakeSubsystem, 1).withTimeout(5));
		// NamedCommands.registerCommand("TurnAndShoot", new
		// TurnAndShootCommand(this.visionSubsystem, this.driveSubsystem,
		// this.armSubsystem, this.shooterSubsystem, this.intakeSubsystem));
		// NamedCommands.registerCommand("Intake", new IntakeCommand(1,
		// this.intakeSubsystem).withTimeout(1));
		// NamedCommands.registerCommand("ArmPickup", new ArmCommand(0,
		// this.armSubsystem));
	}

	// TODO: fix this later to flip correctly based on side color
	// TODO: Check parity of this
	private boolean getAllianceFlipState() {
		return DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == Alliance.Blue : false;
	}

	private void configureBindings() {
		// this.control.presetHigh().onTrue(new ArmCommand(100, this.armSubsystem));
		// this.control.presetLow().onTrue(new ArmCommand(0, this.armSubsystem));

	}

	public Command getAutonomousCommand() {
		Command picked = autoChooser.getSelected();
		if (picked == null) {
			DriverStation.reportError("WARNING: No auto command detected, defaulting to baseline auto.", false);
			return new DriveTimeCommand(1.535, 0.3, driveSubsystem);
		} else {
			return picked;
		}
	}

	public DriveSubsystem getDriveSubsystem() {
		return this.driveSubsystem;
	}
}
