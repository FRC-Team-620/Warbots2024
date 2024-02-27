package org.jmhsrobotics.frc2024.subsystems.shintake.commands;

import org.jmhsrobotics.frc2024.subsystems.shintake.ShintakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ShootRPMCommand extends Command {

	private ShintakeSubsystem shintakeSubsystem;

	private double speed;

	public ShootRPMCommand(ShintakeSubsystem shintakeSubsystem) {
		this.shintakeSubsystem = shintakeSubsystem;

		addRequirements(this.shintakeSubsystem);
	}

	@Override
	public void execute() {
		this.shintakeSubsystem.spinupShooter();
	}

	@Override
	public boolean isFinished() {
		return this.shintakeSubsystem.shooterSpeedAtGoal();
	}

	@Override
	public void end(boolean interrupted) {

	}
}
