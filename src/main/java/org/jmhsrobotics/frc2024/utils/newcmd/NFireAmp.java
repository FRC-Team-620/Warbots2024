package org.jmhsrobotics.frc2024.utils.newcmd;

import org.jmhsrobotics.frc2024.subsystems.intake.IntakeSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem.ControlType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;

public class NFireAmp extends Command {
	ShooterSubsystem shooter;
	IntakeSubsystem intake;
	Debouncer sensorDebounce = new Debouncer(0.5);

	/**
	 * Fires a note into the amp. Runs shooter and flywheel. End when both hasnote
	 * and note too high are false.
	 * 
	 * @param shooter
	 * @param intake
	 */
	public NFireAmp(ShooterSubsystem shooter, IntakeSubsystem intake) {
		this.shooter = shooter;
		this.intake = intake;
		addRequirements(shooter, intake);
	}

	@Override
	public boolean isFinished() {
		return sensorDebounce.calculate(!this.intake.hasNote() && !this.intake.noteTooHigh());
	}

	@Override
	public void initialize() {
		shooter.set(12, ControlType.VOLTAGE);
		intake.set(1);
	}

	@Override
	public void execute() {
		shooter.set(12, ControlType.VOLTAGE);
		intake.set(1);
	}

	@Override
	public void end(boolean interrupted) {
		shooter.set(0, ControlType.VOLTAGE);
		intake.set(0);
	}

}
