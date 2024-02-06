package org.jmhsrobotics.frc2024.autoCommands;

import org.jmhsrobotics.frc2024.subsystems.arm.ArmSubsystem;
import org.jmhsrobotics.frc2024.subsystems.arm.commands.ArmCommand;
import org.jmhsrobotics.frc2024.subsystems.drive.DriveSubsystem;
import org.jmhsrobotics.frc2024.subsystems.drive.commands.LockAprilTag;
import org.jmhsrobotics.frc2024.subsystems.intake.IntakeSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem;
import org.jmhsrobotics.frc2024.subsystems.vision.VisionSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TurnAndShootCommand extends SequentialCommandGroup {

	private VisionSubsystem visionSubsystem;
	private DriveSubsystem driveSubsystem;
	private ArmSubsystem armSubsystem;
	private ShooterSubsystem shooterSubsystem;
	private IntakeSubsystem intakeSubsystem;

	public TurnAndShootCommand(VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem,
			ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
		// TODO: rename the folder name
		this.visionSubsystem = visionSubsystem;
		this.driveSubsystem = driveSubsystem;
		this.armSubsystem = armSubsystem;
		this.shooterSubsystem = shooterSubsystem;
		this.intakeSubsystem = intakeSubsystem;

		addCommands(new ParallelCommandGroup(new LockAprilTag(7, this.driveSubsystem, this.visionSubsystem),
				new ArmCommand(10, this.armSubsystem)).withTimeout(5));
		// new FireCommand(this.intakeSubsystem, this.shooterSubsystem));
		addRequirements(this.visionSubsystem, this.armSubsystem, this.driveSubsystem, this.shooterSubsystem,
				this.intakeSubsystem);
	}

}
