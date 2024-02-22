package org.jmhsrobotics.frc2024.subsystems.intake.commands;

import org.jmhsrobotics.frc2024.subsystems.intake.IntakeSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.wpilibj2.command.Command;

public class DefaultIntakeCommand extends Command {
	private IntakeSubsystem intakeSubsystem;
	private ShooterSubsystem shooterSubsystem;

	private SparkLimitSwitch lowSwitch;
	private SparkLimitSwitch highSwitch;

	private TimeOfFlight lowerSensor;
	private TimeOfFlight upperSensor;

	public DefaultIntakeCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
		this.intakeSubsystem = intakeSubsystem;
		this.shooterSubsystem = shooterSubsystem;

		this.lowSwitch = intakeSubsystem.lowSwitch();
		this.highSwitch = intakeSubsystem.highSwitch();

		this.lowerSensor = this.intakeSubsystem.lowerSensor();
		this.upperSensor = this.intakeSubsystem.upperSensor();
		this.lowerSensor.setRangingMode(RangingMode.Short, 24);
		this.upperSensor.setRangingMode(RangingMode.Short, 24);

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
		boolean hasNote = this.lowerSensor.getRange() < 100;
		boolean noteTooHigh = this.upperSensor.getRange() < 300;

		// boolean lowSwitchPressed = lowSwitch.isPressed();
		// boolean noteTooHigh = highSwitch.isPressed();
		// boolean hasNote = lowSwitch.isPressed();

		if (noteTooHigh) {
			intakeSubsystem.set(-0.1);
			this.shooterSubsystem.setSpeed(-0.1);
		} else {
			intakeSubsystem.set(0);
			this.shooterSubsystem.setSpeed(0);
		}
	}
}
