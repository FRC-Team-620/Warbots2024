package org.jmhsrobotics.frc2024.subsystems.arm.commands;

import org.jmhsrobotics.frc2024.subsystems.arm.ArmPIDSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class CommandArm extends Command {
	private ArmPIDSubsystem armSubsystem;

	// set angle from constants
	private final double angle;

	/**
	 * Moves Arm to specified angle, Command ends when arm has moved to the setpoint
	 * 
	 * @param armSubsystem
	 * @param angleDegrees
	 */
	public CommandArm(ArmPIDSubsystem armSubsystem, double angleDegrees) {
		this.armSubsystem = armSubsystem;
		this.angle = angleDegrees;
		addRequirements(armSubsystem);
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
