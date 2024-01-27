package org.jmhsrobotics.frc2024.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimbCommand extends Command {
	private ClimbSubsystem climbSubsystem;
	private double length;
	private SparkPIDController lengthPIDController;

	public ClimbCommand(ClimbSubsystem climbSubsystem, double length) {
		this.climbSubsystem = climbSubsystem;
		this.length = length;
		addRequirements(climbSubsystem);

		lengthPIDController = climbSubsystem.getMotor().getPIDController();
		lengthPIDController.setFeedbackDevice(climbSubsystem.getEncoder());

		// TODO: tune vals
		lengthPIDController.setP(.1);
		lengthPIDController.setI(0);
		lengthPIDController.setD(0);
		lengthPIDController.setFF(0);
		lengthPIDController.setOutputRange(-1, 1);
		lengthPIDController.setReference(length, CANSparkMax.ControlType.kPosition);
	}

}
