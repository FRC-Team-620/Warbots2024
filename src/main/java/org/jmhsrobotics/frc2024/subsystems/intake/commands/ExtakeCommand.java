package org.jmhsrobotics.frc2024.subsystems.intake.commands;

import org.jmhsrobotics.frc2024.subsystems.intake.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ExtakeCommand extends Command {

	private IntakeSubsystem intakeSubsystem;

	private double speed;

	public ExtakeCommand(IntakeSubsystem intakeSubsystem, double speed) {
		this.speed = speed;
		this.intakeSubsystem = intakeSubsystem;

		addRequirements(this.intakeSubsystem);
	}

	@Override
	public void initialize() {
		this.intakeSubsystem.setIntakeSpeed(0);
	}

	@Override
	public void execute() {
		this.intakeSubsystem.setIntakeSpeed(-this.speed);
	}

	@Override
	public boolean isFinished() {
		return !this.intakeSubsystem.lowSwitch().isPressed() && !this.intakeSubsystem.highSwitch().isPressed();
	}

	@Override
	public void end(boolean interrupted) {
		this.intakeSubsystem.setIntakeSpeed(0);
	}
}
