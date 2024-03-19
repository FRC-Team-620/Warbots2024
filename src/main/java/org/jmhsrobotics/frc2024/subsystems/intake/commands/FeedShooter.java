package org.jmhsrobotics.frc2024.subsystems.intake.commands;

import org.jmhsrobotics.frc2024.subsystems.intake.IntakeSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class FeedShooter extends Command {
	private IntakeSubsystem intakeSubsystem;
	private Timer time = new Timer();
	// private timeOfFlight

	public FeedShooter(IntakeSubsystem intakeSubsystem) {
		this.intakeSubsystem = intakeSubsystem;

		addRequirements(this.intakeSubsystem);
	}

	@Override
	public void initialize() {
		// TODO Auto-generated method stub
		this.intakeSubsystem.set(0);
		time.restart();
	}

	@Override
	public void execute() {
		// TODO Auto-generated method stub
		this.intakeSubsystem.set(1);
		if (this.intakeSubsystem.hasNote()) {
			time.start();
		}
	}

	@Override
	public boolean isFinished() {
		// TODO Auto-generated method stub
		// set timer, finish later
		// return super.isFinished();
		return time.hasElapsed(.5);
	}

	@Override
	public void end(boolean interrupted) {
		// TODO Auto-generated method stub
		this.intakeSubsystem.set(0);
	}

}
