// package org.jmhsrobotics.frc2024.autoCommands;

// import org.jmhsrobotics.frc2024.subsystems.intake.IntakeSubsystem;
// import org.jmhsrobotics.frc2024.subsystems.intake.commands.IntakeCommand;
// import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem;
// import org.jmhsrobotics.frc2024.subsystems.shooter.commands.ShooterCommand;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// public class FireCommand extends SequentialCommandGroup {
// private IntakeSubsystem intakeSubsystem;
// private ShooterSubsystem shooterSubsystem;

// public FireCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem
// shooterSubsystem) {
// this.intakeSubsystem = intakeSubsystem;
// this.shooterSubsystem = shooterSubsystem;

// addCommands(new ShooterCommand(1, this.shooterSubsystem),
// new IntakeCommand(1, this.intakeSubsystem).withTimeout(1));
// }

// }
