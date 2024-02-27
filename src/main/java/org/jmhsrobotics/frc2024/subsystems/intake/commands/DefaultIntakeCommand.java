package org.jmhsrobotics.frc2024.subsystems.intake.commands;

import org.jmhsrobotics.frc2024.subsystems.intake.IntakeSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem.ControlType;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.wpilibj2.command.Command;

public class DefaultIntakeCommand extends Command {
	private IntakeSubsystem intakeSubsystem;
	private ShooterSubsystem shooterSubsystem;

	public DefaultIntakeCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
		this.intakeSubsystem = intakeSubsystem;
		this.shooterSubsystem = shooterSubsystem;

		addRequirements(intakeSubsystem);
	}

	// @Override
	// public void initialize() {}

	@Override
	public void end(boolean interrupted) {
		intakeSubsystem.set(0);
	}

	@Override
	public void execute() {
		boolean hasNote = this.intakeSubsystem.hasNote();
		boolean noteTooHigh = this.intakeSubsystem.noteTooHigh();

		// boolean lowSwitchPressed = lowSwitch.isPressed();
		// boolean noteTooHigh = highSwitch.isPressed();
		// boolean hasNote = lowSwitch.isPressed();

		if (noteTooHigh) {
			intakeSubsystem.set(-0.1);
			this.shooterSubsystem.set(-0.1, ControlType.VOLTAGE);
		} else {
			intakeSubsystem.set(0);
			this.shooterSubsystem.set(0, ControlType.VOLTAGE);
		}
	}
}
