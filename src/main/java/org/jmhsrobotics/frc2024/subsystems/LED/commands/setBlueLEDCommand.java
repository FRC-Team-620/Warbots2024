package org.jmhsrobotics.frc2024.subsystems.LED.commands;

import org.jmhsrobotics.frc2024.subsystems.LED.LEDSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shintake.ShintakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class setBlueLEDCommand extends Command {

	private LEDSubsystem ledSubsystem;
	private ShintakeSubsystem shintakeSubsystem;

	public setBlueLEDCommand(LEDSubsystem ledSubsystem, ShintakeSubsystem shintakeSubsystem) {
		this.ledSubsystem = ledSubsystem;
		this.shintakeSubsystem = shintakeSubsystem;
		addRequirements(this.ledSubsystem);
	}

	@Override
	public void initialize() {
		this.ledSubsystem.startLED();

	}

	@Override
	public void execute() {

		// create a rainbow effect
		this.ledSubsystem.setBlue();
	}

	@Override
	public boolean isFinished() {
		return !this.shintakeSubsystem.hasNote();
	}

	@Override
	public void end(boolean interrupted) {
		this.ledSubsystem.endLED();
	}
}
