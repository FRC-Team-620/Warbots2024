package org.jmhsrobotics.frc2024.ComboCommands;

import org.jmhsrobotics.frc2024.subsystems.shintake.ShintakeSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shintake.ShintakeSubsystem.ControlType;

import edu.wpi.first.wpilibj2.command.Command;

public class AmpShotCommand extends Command {

	private ShintakeSubsystem shintakeSubsystem;
	/**
	 * Runs the intake and the shooter to expell a note into the amp. Command Never
	 * ends
	 *
	 * @param intakeSubsystem
	 * @param shooterSubsystem
	 */
	public AmpShotCommand(ShintakeSubsystem shintakeSubsystem) {
		this.shintakeSubsystem = shintakeSubsystem;

		addRequirements(this.shintakeSubsystem);
	}

	@Override
	public void initialize() {
		this.shintakeSubsystem.setIntakeSpeed(0);
	}

	@Override
	public void execute() {
		this.shintakeSubsystem.setIntakeSpeed(1);
		this.shintakeSubsystem.setShooterGoal(0.4 * 12, ControlType.VOLTAGE);
	}

	@Override
	public boolean isFinished() {
		// return this.intakeSubsystem.hasNote();
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		this.shintakeSubsystem.setIntakeSpeed(0);
	}
}
