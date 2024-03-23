package org.jmhsrobotics.frc2024.subsystems.LED.commands;

import org.jmhsrobotics.frc2024.subsystems.LED.LEDSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ColorLEDCommand extends Command {

	private LEDSubsystem ledSubsystem;
	private int r;
	private int g;
	private int b;

	public ColorLEDCommand(LEDSubsystem ledSubsystem, int r, int g, int b) {
		this.ledSubsystem = ledSubsystem;
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
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		this.ledSubsystem.endLED();
	}
}
