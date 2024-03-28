package org.jmhsrobotics.frc2024.autoCommands;

import org.jmhsrobotics.frc2024.subsystems.shintake.ShintakeSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shintake.ShintakeSubsystem.ControlType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;

public class NSpinupAndShoot extends Command {
	private ShintakeSubsystem shintake;
	private double targetRPM;
	private boolean shouldFire = false;
	private Debouncer noteExitDebouncer = new Debouncer(0.1);

	/**
	 * Spins up Shooter Flywheel to target RPM and then runs intake to shoot a note
	 * into the speaker. Ends when both has note and note too high are false
	 *
	 * @param shintake
	 * @param rpm
	 */
	public NSpinupAndShoot(ShintakeSubsystem shintake, double rpm) {
		this.shintake = shintake;
		this.targetRPM = rpm;
		addRequirements(shintake);
	}

	@Override
	public void initialize() {
		this.shouldFire = false;
		this.shintake.setShooterGoal(targetRPM, ControlType.PID);
	}

	@Override
	public void execute() {
		if (this.shintake.isShooterAtGoal()) { // TODO: Do we need debouncing?
			shouldFire = true;
		}

		shintake.setIntakeSpeed(shouldFire ? 1 : 0);
	}

	@Override
	public boolean isFinished() {
		return shouldFire && noteExitDebouncer.calculate(!this.shintake.hasNote() && !this.shintake.noteTooHigh());
	}

	@Override
	public void end(boolean interrupted) {
		shintake.setIntakeSpeed(0);
		this.shintake.setShooterGoal(targetRPM, ControlType.PID);
	}

}
