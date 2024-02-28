package org.jmhsrobotics.frc2024.ComboCommands;

import org.jmhsrobotics.frc2024.Constants;
import org.jmhsrobotics.frc2024.subsystems.arm.ArmPIDSubsystem;
import org.jmhsrobotics.frc2024.subsystems.arm.commands.CommandArm;
import org.jmhsrobotics.frc2024.subsystems.intake.IntakeSubsystem;
import org.jmhsrobotics.frc2024.subsystems.intake.commands.IntakeCommand;
import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shooter.commands.ShootOpenLoopCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AmpHelper extends SequentialCommandGroup {
	private ArmPIDSubsystem armPIDSubsystem;
	private ShooterSubsystem shooterSubsystem;
	private IntakeSubsystem intakeSubsystem;

	public AmpHelper(ArmPIDSubsystem armPIDSubsystem, ShooterSubsystem shooterSubsystem,
			IntakeSubsystem intakeSubsystem) {
		this.intakeSubsystem = intakeSubsystem;
		this.shooterSubsystem = shooterSubsystem;
		this.armPIDSubsystem = armPIDSubsystem;

		addCommands(new CommandArm(this.armPIDSubsystem, Constants.ArmSetpoint.AMP.value),
				new ShootOpenLoopCommand(1, this.shooterSubsystem).withTimeout(0.5),
				new IntakeCommand(0.8, this.intakeSubsystem, this.shooterSubsystem));
	}
}
