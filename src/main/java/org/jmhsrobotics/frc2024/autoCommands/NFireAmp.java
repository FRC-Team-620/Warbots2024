package org.jmhsrobotics.frc2024.autoCommands;

import org.jmhsrobotics.frc2024.subsystems.shintake.ShintakeSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shintake.ShintakeSubsystem.ControlType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;

public class NFireAmp extends Command {
	ShintakeSubsystem shintakeSubsystem;
	Debouncer sensorDebounce = new Debouncer(0.5);

	/**
	 * Fires a note into the amp. Runs shooter and flywheel. End when both hasnote
	 * and note too high are false.
	 *
	 * @param shooter
	 * @param intake
	 */
	public NFireAmp(ShintakeSubsystem shintakeSubsystem) {
		this.shintakeSubsystem = shintakeSubsystem;
		addRequirements(shintakeSubsystem);
	}

	@Override
	public boolean isFinished() {
		return sensorDebounce.calculate(!this.shintakeSubsystem.hasNote() && !this.shintakeSubsystem.noteTooHigh());
	}

	@Override
	public void initialize() {
		shintakeSubsystem.setShooterGoal(12, ControlType.VOLTAGE);
		shintakeSubsystem.setIntakeSpeed(1);
	}

	@Override
	public void execute() {
		shintakeSubsystem.setShooterGoal(12, ControlType.VOLTAGE);
		shintakeSubsystem.setIntakeSpeed(1);
	}

	@Override
	public void end(boolean interrupted) {
		shintakeSubsystem.setShooterGoal(12, ControlType.VOLTAGE);
		shintakeSubsystem.setIntakeSpeed(0);
	}

}
