package org.jmhsrobotics.frc2024.autoCommands;

import org.jmhsrobotics.frc2024.subsystems.intake.IntakeSubsystem;
import org.jmhsrobotics.frc2024.subsystems.intake.commands.IntakeCommand;
import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shooter.commands.ShootOpenLoopCommand;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class AutoAmpShotCommand extends ParallelCommandGroup {
	private ShooterSubsystem shooterSubsystem;
	private IntakeSubsystem intakeSubsystem;

	public AutoAmpShotCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
		this.shooterSubsystem = shooterSubsystem;
		this.intakeSubsystem = intakeSubsystem;
		addCommands(new ShootOpenLoopCommand(40, this.shooterSubsystem).withTimeout(.75),
				new IntakeCommand(1, this.intakeSubsystem, this.shooterSubsystem).withTimeout(.75));
	}
}
