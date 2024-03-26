package org.jmhsrobotics.frc2024.subsystems.shintake.commands;

import org.jmhsrobotics.frc2024.subsystems.shintake.ShintakeSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shintake.ShintakeSubsystem.ControlType;

import edu.wpi.first.wpilibj2.command.Command;

public class DefaultShintakeCommand extends Command {
	private ShintakeSubsystem shintakeSubsystem;

	public DefaultShintakeCommand(ShintakeSubsystem shintakeSubsystem) {
		this.shintakeSubsystem = shintakeSubsystem;

		addRequirements(this.shintakeSubsystem);
	}

	// @Override
	// public void initialize() {}

	@Override
	public void end(boolean interrupted) {
		this.shintakeSubsystem.setIntakeSpeed(0);
	}

	@Override
	public void execute() {
		boolean hasNote = this.shintakeSubsystem.hasNote();
		boolean noteTooHigh = this.shintakeSubsystem.noteTooHigh();

		// boolean lowSwitchPressed = lowSwitch.isPressed();
		// boolean noteTooHigh = highSwitch.isPressed();
		// boolean hasNote = lowSwitch.isPressed();

		if (noteTooHigh) {
			shintakeSubsystem.setIntakeSpeed(-0.1);
			this.shintakeSubsystem.setShooterGoal(-1.3, ControlType.VOLTAGE);
		} else {
			shintakeSubsystem.setIntakeSpeed(0);
			this.shintakeSubsystem.setShooterGoal(0, ControlType.VOLTAGE);
		}
	}
}
