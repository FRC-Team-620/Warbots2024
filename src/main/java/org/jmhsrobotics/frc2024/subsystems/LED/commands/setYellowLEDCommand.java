package org.jmhsrobotics.frc2024.subsystems.LED.commands;

import org.jmhsrobotics.frc2024.subsystems.LED.LEDSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class setYellowLEDCommand extends Command {

	private LEDSubsystem ledSubsystem;
	private ShooterSubsystem shooterSubsystem;

	public setYellowLEDCommand(LEDSubsystem ledSubsystem, ShooterSubsystem shooterSubsystem) {
		this.ledSubsystem = ledSubsystem;
		this.shooterSubsystem = shooterSubsystem;
		addRequirements(this.ledSubsystem);
	}

	@Override
	public void initialize() {
		this.ledSubsystem.startLED();

	}

	@Override
	public void execute() {

		// create a rainbow effect
		this.ledSubsystem.setYellow();
	}

	@Override
	public boolean isFinished() {
		return !this.shooterSubsystem.atGoal();
	}

	@Override
	public void end(boolean interrupted) {
		this.ledSubsystem.endLED();
	}
}
