package org.jmhsrobotics.frc2024.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.ResourceBundle.Control;

import org.jmhsrobotics.frc2024.Constants;
import org.jmhsrobotics.frc2024.subsystems.arm.ArmPIDSubsystem;


public class ArmSetShootCommand extends Command {

	private ArmPIDSubsystem armSubsystem;

	private double angle = Constants.ArmSetpoint.SHOOT.value;

	public ArmSetShootCommand(ArmPIDSubsystem armSubsystem) {
		this.armSubsystem = armSubsystem;
		addRequirements(this.armSubsystem);

	}

	@Override
	public void initialize() {
		this.armSubsystem.setGoal(this.angle);
	}

	@Override
	public boolean isFinished() {
		// return false;
		return this.armSubsystem.atGoal();
	}
}
