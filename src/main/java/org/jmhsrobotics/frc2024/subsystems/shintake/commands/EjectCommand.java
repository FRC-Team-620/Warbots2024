package org.jmhsrobotics.frc2024.subsystems.shintake.commands;

import org.jmhsrobotics.frc2024.subsystems.shintake.ShintakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class EjectCommand extends Command {

	private ShintakeSubsystem shintakeSubsystem;

	public EjectCommand(ShintakeSubsystem shintakeSubsystem) {
		this.shintakeSubsystem = shintakeSubsystem;

		addRequirements(this.shintakeSubsystem);
	}

	@Override
	public void initialize() {
		this.shintakeSubsystem.stopIntake();
	}

	@Override
	public void execute() {
		this.shintakeSubsystem.ejectNote();
	}

	@Override
	public boolean isFinished() {
		return !this.shintakeSubsystem.hasNote() && !this.shintakeSubsystem.noteTooHigh();
	}

	@Override
	public void end(boolean interrupted) {
		this.shintakeSubsystem.stopIntake();
	}
}
