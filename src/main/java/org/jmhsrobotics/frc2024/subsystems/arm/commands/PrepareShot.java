package org.jmhsrobotics.frc2024.subsystems.arm.commands;

import org.jmhsrobotics.frc2024.subsystems.arm.ArmPIDSubsystem;
import org.jmhsrobotics.frc2024.subsystems.drive.DriveSubsystem;
import org.jmhsrobotics.frc2024.subsystems.drive.commands.LockSpeaker;
import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shooter.commands.ShootOpenLoopCommand;
import org.jmhsrobotics.frc2024.subsystems.vision.VisionSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class PrepareShot extends ParallelCommandGroup {
	private ArmPIDSubsystem arm;
	private VisionSubsystem vision;
	private DriveSubsystem drive;
	private ShooterSubsystem shooter;
	public PrepareShot(DriveSubsystem drive, ArmPIDSubsystem arm, ShooterSubsystem shooter, VisionSubsystem vision) {
		this.drive = drive;
		this.arm = arm;
		this.shooter = shooter;
		this.vision = vision;
		addCommands(new LockSpeaker(drive, vision), new ArmVision(arm, vision, drive),
				new ShootOpenLoopCommand(12, shooter));
	}

	

}
