package org.jmhsrobotics.frc2024.subsystems.LED.commands;

import org.jmhsrobotics.frc2024.subsystems.LED.LEDSubsystem;
import org.jmhsrobotics.frc2024.subsystems.intake.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ColorLEDCommand extends Command {

	private LEDSubsystem ledSubsystem;
	private IntakeSubsystem intakeSubsystem;
	private int r;
	private int g;
	private int b;

	public ColorLEDCommand(LEDSubsystem ledSubsystem, IntakeSubsystem intakeSubsystem, int r, int g, int b) {
		this.ledSubsystem = ledSubsystem;
		this.intakeSubsystem = intakeSubsystem;
		this.r = r;
		this.g = g;
		this.b = b;
		addRequirements(this.ledSubsystem);
	}

	@Override
	public void initialize() {
		this.ledSubsystem.startLED();

	}

	@Override
	public void execute() {

		// create a rainbow effect
		this.ledSubsystem.setRGB(r, g, b);
	}

	@Override
	public boolean isFinished() {
		return !this.intakeSubsystem.hasNote();
	}

	@Override
	public void end(boolean interrupted) {
		this.ledSubsystem.endLED();
	}
}
