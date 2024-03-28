package org.jmhsrobotics.frc2024.subsystems.arm.commands;

import org.jmhsrobotics.frc2024.subsystems.arm.ArmPIDSubsystem;
import org.jmhsrobotics.frc2024.subsystems.drive.DriveSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shintake.ShintakeSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shintake.commands.ShooterAutoCommand;
import org.jmhsrobotics.frc2024.subsystems.vision.VisionSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class PrepareShot extends Command {

	/**
	 * Runs arm vision that automatically moves the arm to the correct shooting
	 * distance and spins up the flywheels to shooting speed.
	 *
	 * @param drive
	 * @param arm
	 * @param shintake
	 * @param vision
	 */
	public PrepareShot(DriveSubsystem drive, ArmPIDSubsystem arm, ShintakeSubsystem shintakeSubsystem, VisionSubsystem vision) {
		Commands.repeatingSequence(new ArmVision(arm, vision, drive), new ShooterAutoCommand(shintakeSubsystem, 5000));
	}
}
