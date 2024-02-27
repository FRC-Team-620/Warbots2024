package org.jmhsrobotics.frc2024.subsystems.shintake.commands;

import org.jmhsrobotics.frc2024.subsystems.shintake.ShintakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {

	private ShintakeSubsystem shintakeSubsystem;

	public IntakeCommand(ShintakeSubsystem shintakeSubsystem) {
		this.shintakeSubsystem = shintakeSubsystem;

		addRequirements(this.shintakeSubsystem);
	}

	@Override
	public void initialize() {
		this.shintakeSubsystem.setIntakeSpeed(0);
	}

	@Override
	public void execute() {
		this.shintakeSubsystem.intake();
		this.shintakeSubsystem.idleShooterForintake();
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		this.shintakeSubsystem.stopIntake();
		this.shintakeSubsystem.stopShooter();
	}
}
