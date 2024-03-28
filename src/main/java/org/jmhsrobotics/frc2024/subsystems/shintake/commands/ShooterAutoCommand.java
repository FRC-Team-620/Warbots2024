package org.jmhsrobotics.frc2024.subsystems.shintake.commands;

import org.jmhsrobotics.frc2024.subsystems.shintake.ShintakeSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shintake.ShintakeSubsystem.ControlType;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterAutoCommand extends Command {
	private ShintakeSubsystem shintakeSubsystem;
	private double targetRPM;

	/**
	 * Spins up shooter to the target rpm. Command never ends, When command is
	 * exited, spin wheels back to zero rpm
	 *
	 * @param shooterSubsystem
	 * @param targetRPM
	 */
	public ShooterAutoCommand(ShintakeSubsystem shintakeSubsystem, double targetRPM) {
		this.shintakeSubsystem = shintakeSubsystem;
		this.targetRPM = targetRPM;
	}

	@Override
	public void execute() {
		this.shintakeSubsystem.setShooterGoal(-1, ControlType.VOLTAGE);
	}

	@Override
	public boolean isFinished() {
		// return this.shooterSubsystem.atGoal();
		return false;

	}

	@Override
	public void end(boolean interrupted) {
		this.shintakeSubsystem.setShooterGoal(0, ControlType.PID);
	}
}
