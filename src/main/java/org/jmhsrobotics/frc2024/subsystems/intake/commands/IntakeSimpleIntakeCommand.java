package org.jmhsrobotics.frc2024.subsystems.intake.commands;

import org.jmhsrobotics.frc2024.subsystems.intake.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeSimpleIntakeCommand extends Command {

	private final IntakeSubsystem intakeSubsystem;

	private double speed;

	/**
	 * Blindly sets intake speed. Command never ends
	 *
	 * @param speed
	 * @param intakeSubsystem
	 */

	public IntakeSimpleIntakeCommand(double speed, IntakeSubsystem intakeSubsystem) {
		this.speed = speed;
		this.intakeSubsystem = intakeSubsystem;

		addRequirements(this.intakeSubsystem);
	}

	@Override
	public void initialize() {
		this.intakeSubsystem.set(0);
	}

	@Override
	public void execute() {
		this.intakeSubsystem.set(this.speed);
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
