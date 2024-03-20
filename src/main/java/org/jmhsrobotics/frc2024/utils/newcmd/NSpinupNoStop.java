package org.jmhsrobotics.frc2024.utils.newcmd;

import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem.ControlType;

import edu.wpi.first.wpilibj2.command.Command;

public class NSpinupNoStop extends Command {
	private ShooterSubsystem shooter;
	private double targetRPM;

	/**
	 * Spins flywheel up to target RPM. Never ends.
	 *
	 * @param shooter
	 * @param rpm
	 */
	public NSpinupNoStop(ShooterSubsystem shooter, double rpm) {
		this.shooter = shooter;
		this.targetRPM = rpm;
		addRequirements(shooter);
	}
	@Override
	public void initialize() {
		shooter.set(targetRPM, ControlType.PID);
	}
	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void execute() {
		this.shooter.set(targetRPM, ControlType.PID);
	}

	@Override
	public void end(boolean interrupted) {
		shooter.set(0, ControlType.PID);
	}

}
