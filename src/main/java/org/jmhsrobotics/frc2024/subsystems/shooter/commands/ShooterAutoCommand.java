package org.jmhsrobotics.frc2024.subsystems.shooter.commands;

import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem.ControlType;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterAutoCommand extends Command {
	private ShooterSubsystem shooterSubsystem;
	private double targetRPM;

	/**
	 * Spins up shooter to the target rpm. Command never ends, When command is
	 * exited, spin wheels back to zero rpm
	 *
	 * @param shooterSubsystem
	 * @param targetRPM
	 */
	public ShooterAutoCommand(ShooterSubsystem shooterSubsystem, double targetRPM) {
		this.shooterSubsystem = shooterSubsystem;
		this.targetRPM = targetRPM;
	}

	@Override
	public void execute() {
		this.shooterSubsystem.set(this.targetRPM, ControlType.PID);
	}

	@Override
	public boolean isFinished() {
		// return this.shooterSubsystem.atGoal();
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		this.shooterSubsystem.set(0, ControlType.PID);
	}
}
