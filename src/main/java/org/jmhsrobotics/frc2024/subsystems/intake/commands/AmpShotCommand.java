package org.jmhsrobotics.frc2024.subsystems.intake.commands;

import org.jmhsrobotics.frc2024.subsystems.intake.IntakeSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem.ControlType;

import edu.wpi.first.wpilibj2.command.Command;

public class AmpShotCommand extends Command {

	private IntakeSubsystem intakeSubsystem;
	private ShooterSubsystem shooterSubsystem;

	public AmpShotCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
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
		this.intakeSubsystem.set(1);
		this.shooterSubsystem.set(0.4 * 12, ControlType.VOLTAGE);
	}

	@Override
	public boolean isFinished() {
		// return this.intakeSubsystem.hasNote();
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		this.intakeSubsystem.set(0);
	}
}
