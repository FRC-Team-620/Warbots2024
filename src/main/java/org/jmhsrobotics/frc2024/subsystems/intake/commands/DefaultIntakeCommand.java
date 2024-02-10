package org.jmhsrobotics.frc2024.subsystems.intake.commands;

import org.jmhsrobotics.frc2024.subsystems.intake.IntakeSubsystem;

import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.wpilibj2.command.Command;

public class DefaultIntakeCommand extends Command {
	private IntakeSubsystem intakeSubsystem;

	private SparkLimitSwitch lowSwitch;
	private SparkLimitSwitch highSwitch;

	public DefaultIntakeCommand(IntakeSubsystem intakeSubsystem) {
		this.intakeSubsystem = intakeSubsystem;
		this.lowSwitch = intakeSubsystem.lowSwitch();
		this.highSwitch = intakeSubsystem.highSwitch();

		addRequirements(intakeSubsystem);
	}

	@Override
	public void initialize() {

	}

	@Override
	public boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		intakeSubsystem.set(0);
	}

	@Override
	public void execute() {
		// boolean lowSwitchPressed = lowSwitch.isPressed();
		boolean highSwitchPressed = highSwitch.isPressed();

		if (!highSwitchPressed) {
			intakeSubsystem.set(0);
		} else {
			intakeSubsystem.set(-.3);
		}
	}
}
