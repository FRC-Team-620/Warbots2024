package org.jmhsrobotics.frc2024.subsystems.shooter.shooterCommands;

import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem;
import org.jmhsrobotics.frc2024.controlBoard.ControlBoard;
import edu.wpi.first.wpilibj2.command.Command;

public class ShooterCommand extends Command {
	private ShooterSubsystem shooterSubsystem;
	private ControlBoard control;

	public ShooterCommand(ShooterSubsystem shooterSubsystem, ControlBoard control) {
		this.shooterSubsystem = shooterSubsystem;
		this.control = control;

		addRequirements(this.shooterSubsystem);
	}

	@Override
	public void execute() {
		double motorSpeed = control.shooterInput();
		shooterSubsystem.setSpeed(motorSpeed);
	}
}
