package org.jmhsrobotics.frc2024.ComboCommands;

import org.jmhsrobotics.frc2024.Constants;
import org.jmhsrobotics.frc2024.subsystems.arm.ArmPIDSubsystem;
import org.jmhsrobotics.frc2024.subsystems.arm.commands.CommandArm;
import org.jmhsrobotics.frc2024.subsystems.shintake.ShintakeSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shintake.commands.IntakeCommand;
import org.jmhsrobotics.frc2024.subsystems.shintake.commands.ShooterAutoCommand;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AmpHelper extends SequentialCommandGroup {
	private ArmPIDSubsystem armPIDSubsystem;
	private ShintakeSubsystem shintakeSubsystem;

	public AmpHelper(ArmPIDSubsystem armPIDSubsystem, ShintakeSubsystem shintakeSubsystem) {
		this.shintakeSubsystem = shintakeSubsystem;
		this.armPIDSubsystem = armPIDSubsystem;

		addCommands(new CommandArm(this.armPIDSubsystem, Constants.ArmSetpoint.AMP.value),
				new ParallelRaceGroup(new ShooterAutoCommand(this.shintakeSubsystem, 5000).withTimeout(5),
						new IntakeCommand(0.8, true, this.shintakeSubsystem).withTimeout(5)));
	}
}
