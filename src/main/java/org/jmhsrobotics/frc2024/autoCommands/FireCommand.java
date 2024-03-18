package org.jmhsrobotics.frc2024.autoCommands;

import org.jmhsrobotics.frc2024.subsystems.intake.IntakeSubsystem;
import org.jmhsrobotics.frc2024.subsystems.intake.commands.IntakeCommand;
import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shooter.commands.ShooterAutoCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FireCommand extends SequentialCommandGroup {
	private IntakeSubsystem intakeSubsystem;
	private ShooterSubsystem shooterSubsystem;

	/**
	 *  Runs the intake for 0.2 seconds and spins up the shooter flywheels to 5000 rpm with a timeout of 0.2 seconds Whole command ends is 0.2 seconds
	 * 
	 * @param intakeSubsystem
	 * @param shooterSubsystem
	 */
	public FireCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
		this.intakeSubsystem = intakeSubsystem;
		this.shooterSubsystem = shooterSubsystem;
		addCommands(new ShooterAutoCommand(this.shooterSubsystem, 5000).withTimeout(0.2),
				new IntakeCommand(1, this.intakeSubsystem, this.shooterSubsystem).withTimeout(0.2));
	}
}
