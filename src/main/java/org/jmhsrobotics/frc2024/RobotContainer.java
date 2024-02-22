// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jmhsrobotics.frc2024;

import org.jmhsrobotics.frc2024.autoCommands.TurnAndShootCommand;
import org.jmhsrobotics.frc2024.controlBoard.CompControl;
import org.jmhsrobotics.frc2024.controlBoard.ControlBoard;
import org.jmhsrobotics.frc2024.subsystems.arm.ArmPIDSubsystem;
import org.jmhsrobotics.frc2024.subsystems.arm.commands.ArmSetAmpCommand;
import org.jmhsrobotics.frc2024.subsystems.arm.commands.ArmSetPickupCommand;
import org.jmhsrobotics.frc2024.subsystems.arm.commands.ArmSetShootCommand;
import org.jmhsrobotics.frc2024.subsystems.climber.ClimberSubsystem;
import org.jmhsrobotics.frc2024.subsystems.drive.DriveSubsystem;
import org.jmhsrobotics.frc2024.subsystems.drive.commands.DriveCommand;
import org.jmhsrobotics.frc2024.subsystems.drive.commands.auto.DriveTimeCommand;
import org.jmhsrobotics.frc2024.subsystems.intake.IntakeSubsystem;
import org.jmhsrobotics.frc2024.subsystems.intake.commands.DefaultIntakeCommand;
import org.jmhsrobotics.frc2024.subsystems.intake.commands.ExtakeCommand;
import org.jmhsrobotics.frc2024.subsystems.intake.commands.IntakeCommand;
import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shooter.commands.ShootOpenLoopCommand;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import monologue.Logged;

public class RobotContainer implements Logged {

	public ControlBoard control = new CompControl();
	// Subsystems
	private final DriveSubsystem driveSubsystem = new DriveSubsystem();

	private final VisionSubsystem visionSubsystem = new VisionSubsystem(this.driveSubsystem);

	private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
	// private final LEDSubsystem ledSubsystem = new LEDSubsystem();
	private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

	private final ArmPIDSubsystem armSubsystem = new ArmPIDSubsystem();

	private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

	private final SendableChooser<Command> autoChooser;

	public RobotContainer() {

		this.driveSubsystem.setDefaultCommand(new DriveCommand(this.driveSubsystem, this.control));
		// this.armSubsystem.setDefaultCommand(new
		// ArmOpenLoopControlCommand(this.armSubsystem, this.control));
		this.intakeSubsystem.setDefaultCommand(new DefaultIntakeCommand(this.intakeSubsystem, this.shooterSubsystem));
		// DefaultIntakeCommand(this.intakeSubsystem));
		// DefaultIntakeCommand(this.intakeSubsystem));
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

		NamedCommands.registerCommand("ArmAmp", new ArmSetAmpCommand(this.armSubsystem));
		NamedCommands.registerCommand("Extake", new ExtakeCommand(this.intakeSubsystem, 1).withTimeout(5));
		NamedCommands.registerCommand("TurnAndShoot", new TurnAndShootCommand(this.visionSubsystem, this.driveSubsystem,
				this.armSubsystem, this.shooterSubsystem, this.intakeSubsystem));
		NamedCommands.registerCommand("Intake",
				new IntakeCommand(1, this.intakeSubsystem, this.shooterSubsystem).withTimeout(1));
		NamedCommands.registerCommand("ArmPickup", new ArmSetPickupCommand(this.armSubsystem));
	}

	// TODO: fix this later to flip correctly based on side color
	// TODO: Check parity of this
	private boolean getAllianceFlipState() {
		return DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == Alliance.Blue : false;
	}

	private void configureBindings() {
		this.control.presetHigh().onTrue(new ArmSetAmpCommand(this.armSubsystem));
		this.control.presetMid().onTrue(new ArmSetShootCommand(this.armSubsystem));
		this.control.presetLow().onTrue(new ArmSetPickupCommand(this.armSubsystem));
		this.control.intakeInput().whileTrue(new IntakeCommand(0.5, this.intakeSubsystem, this.shooterSubsystem));
		this.control.extakeInput().whileTrue(new IntakeCommand(-0.5, this.intakeSubsystem, this.shooterSubsystem));
		this.control.shooterInput().whileTrue(new ShootOpenLoopCommand(80, shooterSubsystem));

		// temp climber controls
		this.control.climberExtend().whileTrue(new InstantCommand(climberSubsystem::extend));

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
