package org.jmhsrobotics.frc2024.subsystems.intake.commands;

import org.jmhsrobotics.frc2024.subsystems.intake.IntakeSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem;

import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.wpilibj2.command.Command;

public class DefaultIntakeCommand extends Command {
	private IntakeSubsystem intakeSubsystem;
	private ShooterSubsystem shooterSubsystem;

	private SparkLimitSwitch lowSwitch;
	private SparkLimitSwitch highSwitch;

	public DefaultIntakeCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
		this.intakeSubsystem = intakeSubsystem;
		this.shooterSubsystem = shooterSubsystem;

		this.lowSwitch = intakeSubsystem.lowSwitch();
		this.highSwitch = intakeSubsystem.highSwitch();

		addRequirements(intakeSubsystem);
	}

	@Override
	public void initialize() {

	}

	@Override
	public void end(boolean interrupted) {
		intakeSubsystem.set(0);
	}

	@Override
	public void execute() {
		// boolean lowSwitchPressed = lowSwitch.isPressed();
		boolean noteTooHigh = highSwitch.isPressed();
		boolean hasNote = lowSwitch.isPressed();

		if (noteTooHigh && hasNote) {
			intakeSubsystem.set(-.3);
			this.shooterSubsystem.setSpeed(-0.15);
		} else {
			intakeSubsystem.set(0);
			this.shooterSubsystem.setSpeed(0);
		}
	}
}
