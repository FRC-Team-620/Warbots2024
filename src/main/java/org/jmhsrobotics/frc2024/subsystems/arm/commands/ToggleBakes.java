package org.jmhsrobotics.frc2024.subsystems.arm.commands;

import org.jmhsrobotics.frc2024.subsystems.arm.ArmPIDSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ToggleBakes extends Command {

	private ArmPIDSubsystem arm;
	public ToggleBakes(ArmPIDSubsystem arm) {
		this.arm = arm;
	}

	@Override
	public void initialize() {
		this.arm.toggleBakes();
	}
	@Override
	public boolean runsWhenDisabled() {
		return true;
	}
	@Override
	public boolean isFinished() {
		return true;
	}

}
