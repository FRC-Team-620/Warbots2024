package org.jmhsrobotics.frc2024.subsystems.shintake.commands;

import org.jmhsrobotics.frc2024.subsystems.shintake.ShintakeSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shintake.ShintakeSubsystem.ControlType;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {

	private final ShintakeSubsystem shintakeSubsystem;

	private double speed;
	private boolean isFiring = false;
	/**
	 * Blindly Intakes while running the shooter motor backwards. Command never ends
	 *
	 * @param speed
	 * @param shitnakeSubsystem
	 */

	public IntakeCommand(double speed, boolean isFiring, ShintakeSubsystem shintakeSubsystem) { // Fixme:
																				// add
																				// requirements
																				// for
																				// shooter!
		this.speed = speed;
		this.shintakeSubsystem = shintakeSubsystem;
		this.isFiring = isFiring;

		addRequirements(this.shintakeSubsystem);
	}

	@Override
	public void initialize() {
		this.shintakeSubsystem.setIntakeSpeed(0);
	}

	@Override
	public void execute() {
		if(this.isFiring){
			this.shintakeSubsystem.setIntakeSpeed(this.speed);
		}else{
			this.shintakeSubsystem.setIntakeSpeed(this.speed);
			this.shintakeSubsystem.setShooterGoal(-1, ControlType.VOLTAGE);
	}
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
