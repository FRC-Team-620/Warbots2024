package org.jmhsrobotics.frc2024.ComboCommands;

import org.jmhsrobotics.frc2024.Constants;
import org.jmhsrobotics.frc2024.subsystems.intake.IntakeSubsystem;
import org.jmhsrobotics.frc2024.subsystems.intake.commands.IntakeSimpleIntakeCommand;
import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shooter.commands.ShooterAutoCommand;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class ComboEjectCommand extends ParallelCommandGroup {

	/**
	 * Intakes, reversing the shooter wheels
	 *
	 * @param shooter
	 * @param intake
	 */
	public ComboEjectCommand(ShooterSubsystem shooter, IntakeSubsystem intake) {
		addCommands(new IntakeSimpleIntakeCommand(Constants.Intake.eject, intake), // start intake
				new ShooterAutoCommand(shooter, Constants.Shooter.ejectSpeed) // set shooter to intake speed
		);
	}

}
