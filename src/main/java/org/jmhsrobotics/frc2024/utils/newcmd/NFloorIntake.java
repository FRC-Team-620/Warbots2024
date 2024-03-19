package org.jmhsrobotics.frc2024.utils.newcmd;

import org.jmhsrobotics.frc2024.Constants;
import org.jmhsrobotics.frc2024.subsystems.arm.ArmPIDSubsystem;
import org.jmhsrobotics.frc2024.subsystems.arm.commands.CommandArm;
import org.jmhsrobotics.frc2024.subsystems.intake.IntakeSubsystem;

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
    public NFloorIntake(ArmPIDSubsystem arm, IntakeSubsystem intake) {
        // new CommandArm(arm, Constants.ArmSetpoint.PICKUP.value),
        addCommands(Commands.either(Commands.none(),
                Commands.parallel(
                        new CommandArm(arm, Constants.ArmSetpoint.PICKUP.value),
                        new LIntake(intake)),
                intake::hasNote));

    }

    private class LIntake extends Command {
        private Debouncer debounce = new Debouncer(0.04); // TODO: Tune
        private IntakeSubsystem intake;

        public LIntake(IntakeSubsystem intake) {
            this.intake = intake;
            addRequirements(intake);
        }

        @Override
        public void execute() {
            intake.set(1);
        }

        @Override
        public boolean isFinished() {
            return debounce.calculate(intake.hasNote());
        }

        @Override
        public void end(boolean interrupted) {
            intake.set(0);
        }
    }

}
