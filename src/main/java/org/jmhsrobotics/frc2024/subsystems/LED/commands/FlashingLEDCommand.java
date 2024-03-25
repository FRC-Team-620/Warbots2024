package org.jmhsrobotics.frc2024.subsystems.LED.commands;

import org.jmhsrobotics.frc2024.subsystems.LED.LEDSubsystem;
import org.jmhsrobotics.frc2024.subsystems.intake.IntakeSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class FlashingLEDCommand extends Command {

	private LEDSubsystem ledSubsystem;
	private IntakeSubsystem intakeSubsystem;
	private Timer timer;

	public FlashingLEDCommand(LEDSubsystem ledSubsystem, IntakeSubsystem intakeSubsystem) {
		this.ledSubsystem = ledSubsystem;
		this.intakeSubsystem = intakeSubsystem;
		this.timer = new Timer();

		addRequirements(this.ledSubsystem);
	}

	@Override
	public void initialize() {
		this.timer.reset();
		this.timer.start();
		this.ledSubsystem.startLED();

	}

	@Override
	public void execute() {
		int ms = (int) (Timer.getFPGATimestamp() * 1000);
		if (ms % 200 > 100) {
			this.ledSubsystem.setGreen();
		} else {
			this.ledSubsystem.setRed();
		}
	}

	@Override
	public boolean isFinished() {
		return !this.intakeSubsystem.isIntaking();
	}

	@Override
	public void end(boolean interrupted) {

		this.ledSubsystem.endLED();
	}
}
