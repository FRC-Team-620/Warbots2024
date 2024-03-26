package org.jmhsrobotics.frc2024.utils.newcmd;

import org.jmhsrobotics.frc2024.subsystems.shintake.ShintakeSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shintake.ShintakeSubsystem.ControlType;

import edu.wpi.first.wpilibj2.command.Command;

public class NSpinupNoStop extends Command {
	private ShintakeSubsystem shintake;
	private double targetRPM;

	/**
	 * Spins flywheel up to target RPM. Never ends.
	 *
	 * @param shintake
	 * @param rpm
	 */
	public NSpinupNoStop(ShintakeSubsystem shintake, double rpm) {
		this.shintake = shintake;
		this.targetRPM = rpm;
		addRequirements(shintake);
	}
	@Override
	public void initialize() {
		shintake.setShooterGoal(targetRPM, ControlType.PID);
	}
	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void execute() {
		this.shintake.setShooterGoal(targetRPM, ControlType.PID);
	}

	@Override
	public void end(boolean interrupted) {
		shintake.setShooterGoal(0, ControlType.PID);
	}

}
