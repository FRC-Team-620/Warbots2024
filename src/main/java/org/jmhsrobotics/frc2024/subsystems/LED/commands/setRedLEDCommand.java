package org.jmhsrobotics.frc2024.subsystems.LED.commands;

import org.jmhsrobotics.frc2024.subsystems.LED.LEDSubsystem;
import org.jmhsrobotics.frc2024.subsystems.intake.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class setRedLEDCommand extends Command {

	private LEDSubsystem ledSubsystem;
	private IntakeSubsystem intakeSubsystem;

	public setRedLEDCommand(LEDSubsystem ledSubsystem, IntakeSubsystem intakeSubsystem) {
		this.ledSubsystem = ledSubsystem;
		this.intakeSubsystem = intakeSubsystem;
		addRequirements(this.ledSubsystem);
	}

	@Override
	public void initialize() {
		this.ledSubsystem.startLED();

	}

	@Override
	public void execute() {

		// create a rainbow effect
		this.ledSubsystem.setRed();
	}

	@Override
	public boolean isFinished() {
		return !this.intakeSubsystem.noteTooHigh();
	}

	@Override
	public void end(boolean interrupted) {
		this.ledSubsystem.endLED();
	}
}
