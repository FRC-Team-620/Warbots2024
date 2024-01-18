package org.jmhsrobotics.frc2024.subsystems.LED.commands;

import org.jmhsrobotics.frc2024.subsystems.LED.LEDSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class LEDCommand extends Command {

	private LEDSubsystem ledSubsystem;

	public LEDCommand(LEDSubsystem ledSubsystem) {
		this.ledSubsystem = ledSubsystem;

		addRequirements(this.ledSubsystem);
	}

	@Override
	public void initialize() {
		this.ledSubsystem.startLED();
		
	}

	@Override
	public void execute() {

		// create a rainbow effect
		// this.ledSubsystem.rainBow();
        this.ledSubsystem.setRGB(255, 0, 0);
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
