package org.jmhsrobotics.frc2024.subsystems.intake.commands;

import org.jmhsrobotics.frc2024.subsystems.intake.IntakeSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem.ControlType;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoIntakeCommand extends Command {

	private IntakeSubsystem intakeSubsystem;
	private ShooterSubsystem shooterSubsystem;

	private double speed;

	public AutoIntakeCommand(double speed, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
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
		this.shooterSubsystem.set(-.05, ControlType.VOLTAGE);
	}

	@Override
	public boolean isFinished() {
		return this.intakeSubsystem.hasNote();
		// return false;
	}

	@Override
	public void end(boolean interrupted) {
		this.intakeSubsystem.set(0);
	}
}
