package org.jmhsrobotics.frc2024.autoCommands;

import org.jmhsrobotics.frc2024.subsystems.intake.IntakeSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem.ControlType;

import edu.wpi.first.wpilibj2.command.Command;

public class ScoreAmp extends Command {
	private IntakeSubsystem intake;
	private ShooterSubsystem shooter;
	public ScoreAmp(IntakeSubsystem intake, ShooterSubsystem shooter) {
		this.intake = intake;
		this.shooter = shooter;
		addRequirements(this.intake, this.shooter);
	}

	@Override
	public void initialize() {
		this.intake.set(1);
		this.shooter.set(12, ControlType.VOLTAGE);
	}

	@Override
	public void execute() {
		this.intake.set(1);
		this.shooter.set(12, ControlType.VOLTAGE);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
