package org.jmhsrobotics.frc2024.subsystems.intake.commands;

import org.jmhsrobotics.frc2024.Constants;
import org.jmhsrobotics.frc2024.subsystems.intake.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class DefaultIntakeCommand extends Command {
	private IntakeSubsystem intakeSubsystem;

	public DefaultIntakeCommand(IntakeSubsystem intakeSubsystem) {
		this.intakeSubsystem = intakeSubsystem;

		addRequirements(intakeSubsystem);
	}

	// @Override
	// public void initialize() {}

	@Override
	public void end(boolean interrupted) {
		intakeSubsystem.stop();
	}

	@Override
	public void execute() {
		// boolean hasNote = this.intakeSubsystem.hasNote();
		boolean noteTooHigh = this.intakeSubsystem.noteTooHigh();

		// boolean lowSwitchPressed = lowSwitch.isPressed();
		// boolean noteTooHigh = highSwitch.isPressed();
		// boolean hasNote = lowSwitch.isPressed();

		if (noteTooHigh) {
			intakeSubsystem.set(Constants.Intake.adjustSpeed);
			// this.shooterSubsystem.set(-0.1, ControlType.VOLTAGE);
		} else {
			intakeSubsystem.stop();
			// this.shooterSubsystem.set(0, ControlType.VOLTAGE);
		}
	}
}
