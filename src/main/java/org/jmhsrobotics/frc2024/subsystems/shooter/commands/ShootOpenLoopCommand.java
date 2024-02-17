package org.jmhsrobotics.frc2024.subsystems.shooter.commands;

import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ShootOpenLoopCommand extends Command {

	private ShooterSubsystem shooterSubsystem;

	private double speed;

	public ShootOpenLoopCommand(double speed, ShooterSubsystem shooterSubsystem) {
		this.speed = speed;
		this.shooterSubsystem = shooterSubsystem;

		addRequirements(this.shooterSubsystem);
	}

	@Override
	public void initialize() {
		this.shooterSubsystem.setSpeed(speed);
	}

	@Override
	public void execute() {
		this.shooterSubsystem.setSpeed(speed);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		this.shooterSubsystem.setSpeed(0);
	}
}
