package org.jmhsrobotics.frc2024.autoCommands;

import org.jmhsrobotics.frc2024.subsystems.intake.IntakeSubsystem;
import org.jmhsrobotics.frc2024.subsystems.intake.commands.IntakeCommand;
import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shooter.commands.ShootOpenLoopCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FireCommand extends SequentialCommandGroup {
	private IntakeSubsystem intakeSubsystem;
	private ShooterSubsystem shooterSubsystem;

	public FireCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
		this.intakeSubsystem = intakeSubsystem;
		this.shooterSubsystem = shooterSubsystem;
		addCommands(new ShootOpenLoopCommand(80, this.shooterSubsystem).withTimeout(1),
				new IntakeCommand(1, this.intakeSubsystem, this.shooterSubsystem).withTimeout(0.2));
	}
}
