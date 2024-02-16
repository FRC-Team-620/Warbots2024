package org.jmhsrobotics.frc2024.subsystems.intake.commands;

import org.jmhsrobotics.frc2024.subsystems.intake.IntakeSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {

	private IntakeSubsystem intakeSubsystem;
	private ShooterSubsystem shooterSubsystem;

	private double speed;

	public IntakeCommand(double speed, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
		this.speed = speed;
		this.intakeSubsystem = intakeSubsystem;
		this.shooterSubsystem = shooterSubsystem;

		addRequirements(this.intakeSubsystem);
	}

	@Override
	public void initialize() {
		this.intakeSubsystem.set(0);
	}

	@Override
	public void execute() {
		this.intakeSubsystem.set(this.speed);
		this.shooterSubsystem.setSpeed(-0.1);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		this.intakeSubsystem.set(0);
	}
}
