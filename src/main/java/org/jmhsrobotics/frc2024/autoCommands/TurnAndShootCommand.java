package org.jmhsrobotics.frc2024.autoCommands;

import org.jmhsrobotics.frc2024.subsystems.arm.ArmPIDSubsystem;
import org.jmhsrobotics.frc2024.subsystems.arm.commands.ArmSetShootCommand;
import org.jmhsrobotics.frc2024.subsystems.drive.DriveSubsystem;
import org.jmhsrobotics.frc2024.subsystems.drive.commands.LockAprilTag;
import org.jmhsrobotics.frc2024.subsystems.shintake.ShintakeSubsystem;
import org.jmhsrobotics.frc2024.subsystems.vision.VisionSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TurnAndShootCommand extends SequentialCommandGroup {

	private VisionSubsystem visionSubsystem;
	private DriveSubsystem driveSubsystem;
	private ArmPIDSubsystem armSubsystem;
	private ShintakeSubsystem shintakeSubsystem;

	public TurnAndShootCommand(VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem,
			ArmPIDSubsystem armSubsystem, ShintakeSubsystem shintakeSubsystem) {
		// TODO: rename the folder name
		this.visionSubsystem = visionSubsystem;
		this.driveSubsystem = driveSubsystem;
		this.armSubsystem = armSubsystem;
		this.shintakeSubsystem = shintakeSubsystem;

		addCommands(new ParallelCommandGroup(new LockAprilTag(7, this.driveSubsystem, this.visionSubsystem),
				new ArmSetShootCommand(this.armSubsystem)), new FireCommand(this.shintakeSubsystem));

		addRequirements(this.visionSubsystem, this.armSubsystem, this.driveSubsystem, this.shintakeSubsystem);
	}

}
