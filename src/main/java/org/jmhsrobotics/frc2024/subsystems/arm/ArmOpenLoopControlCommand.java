package org.jmhsrobotics.frc2024.subsystems.arm;

import org.jmhsrobotics.frc2024.controlBoard.ControlBoard;

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
		// TODO Auto-generated method stub
		this.armSubsystem.setArmPivot(0);

	}
	@Override
	public void execute() {
		// TODO Auto-generated method stub
		armSubsystem.setArmPivot(control.pitchInput());
	}

}
