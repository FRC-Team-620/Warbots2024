package org.jmhsrobotics.frc2024.subsystems.shintake.commands;

import org.jmhsrobotics.frc2024.subsystems.shintake.ShintakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeFireCommand extends Command {

	private ShintakeSubsystem shintakeSubsystem;

	private double speed;

	public IntakeFireCommand(double speed, ShintakeSubsystem shintakeSubsystem) {
		this.speed = speed;
		this.shintakeSubsystem = shintakeSubsystem;

		addRequirements(this.shintakeSubsystem);
	}

	@Override
	public void initialize() {
		this.shintakeSubsystem.setIntakeSpeed(0);
	}

	@Override
	public void execute() {
		this.shintakeSubsystem.setIntakeSpeed(this.speed);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		this.shintakeSubsystem.setIntakeSpeed(0);
	}
}
