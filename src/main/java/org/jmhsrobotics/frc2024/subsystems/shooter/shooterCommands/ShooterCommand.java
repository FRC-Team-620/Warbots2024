package org.jmhsrobotics.frc2024.subsystems.shooter.shooterCommands;

import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class ShooterCommand extends Command {
	private ShooterSubsystem shooter;
	private double targetRPM;
	private PIDController pid;

	public ShooterCommand(double targetRPM, ShooterSubsystem shooterSubsystem) {
		this.shooter = shooterSubsystem;
		this.targetRPM = targetRPM;
		pid = new PIDController(.1, 0, 0);

		addRequirements(this.shooter);
	}

	@Override
	public void initialize() {
		pid.reset();
		pid.setSetpoint(targetRPM);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		shooter.setSpeed(0);
	}

	@Override
	public void execute() {
		double motorSpeed = pid.calculate(shooter.getRPM(), targetRPM);
		shooter.setSpeed(motorSpeed);
	}
}
