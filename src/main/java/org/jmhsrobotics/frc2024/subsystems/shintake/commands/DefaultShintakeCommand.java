package org.jmhsrobotics.frc2024.subsystems.shintake.commands;

import org.jmhsrobotics.frc2024.subsystems.shintake.ShintakeSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shintake.ShintakeSubsystem.ShooterControlType;

import edu.wpi.first.wpilibj2.command.Command;

public class DefaultShintakeCommand extends Command {
	private ShintakeSubsystem shintakeSubsystem;

	public DefaultShintakeCommand(ShintakeSubsystem shintakeSubsystem) {
		this.shintakeSubsystem = shintakeSubsystem;

		addRequirements(shintakeSubsystem);
	}

	// @Override
	// public void initialize() {}

	@Override
	public void end(boolean interrupted) {
		shintakeSubsystem.setIntakeSpeed(0);
	}

	@Override
	public void execute() {
		boolean hasNote = this.shintakeSubsystem.hasNote();
		boolean noteTooHigh = this.shintakeSubsystem.noteTooHigh();

		// boolean lowSwitchPressed = lowSwitch.isPressed();
		// boolean noteTooHigh = highSwitch.isPressed();
		// boolean hasNote = lowSwitch.isPressed();

		if (noteTooHigh) {
			// bring note back down and spin the shooter heels backwards so it doesnt poop
			// out
			this.shintakeSubsystem.setIntakeSpeed(-0.1);
			this.shintakeSubsystem.setShooterSpeed(-0.1, ShooterControlType.VOLTAGE);
		} else {
			// turn everything off
			this.shintakeSubsystem.setIntakeSpeed(0);
			this.shintakeSubsystem.setShooterSpeed(0, ShooterControlType.VOLTAGE);
		}
	}
}
