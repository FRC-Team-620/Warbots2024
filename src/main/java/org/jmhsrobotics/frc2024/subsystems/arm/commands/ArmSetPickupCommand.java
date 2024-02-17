package org.jmhsrobotics.frc2024.subsystems.arm.commands;

import org.jmhsrobotics.frc2024.Constants.ArmSetpoint;
import org.jmhsrobotics.frc2024.subsystems.arm.ArmPIDSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ArmSetPickupCommand extends Command {
	private ArmPIDSubsystem armSubsystem;
	private double angle = ArmSetpoint.PICKUP.value;

	public ArmSetPickupCommand(ArmPIDSubsystem armSubsystem) {
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
