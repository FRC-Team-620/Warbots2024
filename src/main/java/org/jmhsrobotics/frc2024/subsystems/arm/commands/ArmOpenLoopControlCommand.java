package org.jmhsrobotics.frc2024.subsystems.arm.commands;

import org.jmhsrobotics.frc2024.controlBoard.ControlBoard;
import org.jmhsrobotics.frc2024.subsystems.arm.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ArmOpenLoopControlCommand extends Command {

	ArmSubsystem armSubsystem;
	ControlBoard control;

	public ArmOpenLoopControlCommand(ArmSubsystem armSubsystem, ControlBoard control) {
		this.armSubsystem = armSubsystem;
		this.control = control;
		addRequirements(armSubsystem);
	}

	@Override
	public void initialize() {
		this.armSubsystem.setArmPivot(0);

	}
	@Override
	public void execute() {
		armSubsystem.setArmPivot(control.pitchInput() * 0.15);
	}

	@Override
	public void end(boolean interrupted) {
		// TODO Auto-generated method stub
		armSubsystem.setArmPivot(0);
	}
}
