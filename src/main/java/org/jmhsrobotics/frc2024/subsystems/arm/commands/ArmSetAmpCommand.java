package org.jmhsrobotics.frc2024.subsystems.arm.commands;

import org.jmhsrobotics.frc2024.Constants.ArmSetpoint;
import org.jmhsrobotics.frc2024.subsystems.arm.ArmPIDSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ArmSetAmpCommand extends Command {
	private ArmPIDSubsystem armSubsystem;

	// set angle from constants
	private double angle = ArmSetpoint.AMP.value;

	public ArmSetAmpCommand(ArmPIDSubsystem armSubsystem) {
		this.armSubsystem = armSubsystem;
	}

	@Override
	public void initialize() {
		this.armSubsystem.setGoal(this.angle);
	}

	@Override
	public boolean isFinished() {
		return this.armSubsystem.atGoal();
	}
}
