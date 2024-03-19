package org.jmhsrobotics.frc2024.utils.newcmd;

import org.jmhsrobotics.frc2024.subsystems.intake.IntakeSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem.ControlType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;

public class NSpinupAndShoot extends Command {

	private ShooterSubsystem shooter;
	private IntakeSubsystem intake;
	private double targetRPM;
	private boolean shouldFire = false;
	private Debouncer noteExitDebouncer = new Debouncer(0.1);

	/**
	 * Spins up Shooter Flywheel to target RPM and then runs intake to shoot a note
	 * into the speaker. Ends when both has note and note too high are false
	 *
	 * @param shooter
	 * @param intake
	 * @param rpm
	 */
	public NSpinupAndShoot(ShooterSubsystem shooter, IntakeSubsystem intake, double rpm) {
		this.shooter = shooter;
		this.intake = intake;
		this.targetRPM = rpm;
		addRequirements(shooter, intake);
	}

	@Override
	public void initialize() {
		this.shouldFire = false;
		this.shooter.set(targetRPM, ControlType.PID);
	}

	@Override
	public void execute() {
		if (this.shooter.atGoal()) { // TODO: Do we need debouncing?
			shouldFire = true;
		}

		intake.set(shouldFire ? 1 : 0);
	}

	@Override
	public boolean isFinished() {
		return shouldFire && noteExitDebouncer.calculate(!this.intake.hasNote() && !this.intake.noteTooHigh());
	}

	@Override
	public void end(boolean interrupted) {
		intake.set(0);
		shooter.set(0, ControlType.PID);
	}

}
