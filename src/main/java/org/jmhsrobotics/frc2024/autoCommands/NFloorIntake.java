package org.jmhsrobotics.frc2024.autoCommands;

import org.jmhsrobotics.frc2024.Constants;
import org.jmhsrobotics.frc2024.subsystems.arm.ArmPIDSubsystem;
import org.jmhsrobotics.frc2024.subsystems.arm.commands.CommandArm;
import org.jmhsrobotics.frc2024.subsystems.shintake.ShintakeSubsystem;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class NFloorIntake extends SequentialCommandGroup {
	/**
	 * Moves the arm to the intake position and runs the intake. Runs untill a note
	 * is detected
	 *
	 * @param arm
	 * @param intake
	 */
	public NFloorIntake(ArmPIDSubsystem arm, ShintakeSubsystem shintakeSubsystem) {
		// new CommandArm(arm, Constants.ArmSetpoint.PICKUP.value),
		addCommands(Commands.either(Commands.none(), Commands
				.parallel(new CommandArm(arm, Constants.ArmSetpoint.PICKUP.value), new LIntake(shintakeSubsystem)),
				shintakeSubsystem::hasNote));

	}

	private class LIntake extends Command {
		private Debouncer debounce = new Debouncer(0.04); // TODO: Tune
		private ShintakeSubsystem shintakeSubsystem;

		public LIntake(ShintakeSubsystem shintakeSubsystem) {
			this.shintakeSubsystem = shintakeSubsystem;
			addRequirements(shintakeSubsystem);
		}

		@Override
		public void execute() {
			shintakeSubsystem.setIntakeSpeed(1);
		}

		@Override
		public boolean isFinished() {
			return debounce.calculate(shintakeSubsystem.hasNote());
		}

		@Override
		public void end(boolean interrupted) {
			shintakeSubsystem.setIntakeSpeed(0);
		}
	}

}
