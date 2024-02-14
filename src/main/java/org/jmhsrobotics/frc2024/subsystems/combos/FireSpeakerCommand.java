package org.jmhsrobotics.frc2024.subsystems.combos;

import org.jmhsrobotics.frc2024.subsystems.arm.ArmPIDSubsystem;
import org.jmhsrobotics.frc2024.subsystems.arm.commands.ArmSetShootCommand;
import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shooter.commands.ShooterCommand;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class FireSpeakerCommand extends ParallelCommandGroup{
    private ArmPIDSubsystem armSubsystem;
    private ShooterSubsystem shooterSubsystem;

    public FireSpeakerCommand(ShooterSubsystem shooterSubsystem, ArmPIDSubsystem armPIDSubsystem){
        this.armSubsystem = armSubsystem;
        this.shooterSubsystem = shooterSubsystem;

        addCommands(new ArmSetShootCommand(this.armSubsystem), new ShooterCommand(1, this.shooterSubsystem));
        addRequirements(this.armSubsystem, this.shooterSubsystem);
    }
}
