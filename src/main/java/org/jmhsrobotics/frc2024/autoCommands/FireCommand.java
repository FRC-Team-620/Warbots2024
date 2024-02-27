package org.jmhsrobotics.frc2024.autoCommands;

import org.jmhsrobotics.frc2024.subsystems.shintake.ShintakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class FireCommand extends Command {
	private ShintakeSubsystem shintakeSubsystem;
	// TODO change this to whatever shooter speed you need
	private double shooterRPM = 100;

	public FireCommand(ShintakeSubsystem shintakeSubsystem) {
		this.shintakeSubsystem = shintakeSubsystem;
	}

	@Override
	public void initialize() {
		// start spinup so we can check it during execute
		this.shintakeSubsystem.spinupShooter();
	}

	@Override
	public void execute() {
		// my wheels are not up to speed and i have a note
		boolean inSpinup = !this.shintakeSubsystem.shooterSpeedAtGoal() && this.shintakeSubsystem.hasNote();
		// my whels are up to speed and i have a note
		boolean atSpeedAndHasNotShot = this.shintakeSubsystem.shooterSpeedAtGoal() && this.shintakeSubsystem.hasNote();

		if (inSpinup) { // get shooter wheels up to speed
			this.shintakeSubsystem.spinupShooter();
		} else if (atSpeedAndHasNotShot) { // shoot the ring until you dont have it anymore
			this.shintakeSubsystem.fire();
		}

	}

	@Override
	public boolean isFinished() {
		return this.shintakeSubsystem.hasNoNote();
	}

	@Override
	public void end(boolean interrupted) {
		// turn off the wheels
		this.shintakeSubsystem.stopIntake();
		this.shintakeSubsystem.stopShooter();
	}

}
