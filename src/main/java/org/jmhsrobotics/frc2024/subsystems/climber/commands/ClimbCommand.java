package org.jmhsrobotics.frc2024.subsystems.climber.commands;

import org.jmhsrobotics.frc2024.subsystems.climber.ClimberSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimbCommand extends Command {
	ClimberSubsystem climberSubsystem;
	ProfiledPIDController climberPID;
	Constraints climberConstraint;
	// in encodercount
	double positionGoal;

	public ClimbCommand(ClimberSubsystem climberSubsystem, double positionGoal) {
		this.climberSubsystem = climberSubsystem;
		this.climberConstraint = new Constraints(80, 120);
		this.climberPID = new ProfiledPIDController(0.001, 0, 0, this.climberConstraint);
		this.positionGoal = positionGoal;
		SmartDashboard.putData("ClimberPID", this.climberPID);
		addRequirements(this.climberSubsystem);
	}

	@Override
	public void initialize() {
		this.climberPID.reset(new State(this.positionGoal, 0));
		this.climberPID.setGoal(this.positionGoal);
		this.climberPID.setTolerance(2, 3);
	}

	@Override
	public void execute() {
		// calculate motor output from pid controller

		this.climberPID.setConstraints(new Constraints(80, 120));
		double motorRawOutput = this.climberPID.calculate(this.climberSubsystem.getEncoderPostition());
		double powerLim = 0.3;
		double motorOutput = MathUtil.clamp(motorRawOutput, -powerLim, powerLim);

		// pass motor output to motor in subsystem
		this.climberSubsystem.set(motorOutput);
		// SmartDashboard.putNumber("ExtensionPID/output", motorOutput);
		// SmartDashboard.putNumber("ExtensionPID/setpoint",
		// this.climberPID.getSetpoint().position); // 85 is max

		SmartDashboard.putNumber("ClimberPID/positionError", this.climberPID.getPositionError());
		SmartDashboard.putNumber("ClimberPID/velocityError", this.climberPID.getVelocityError());
	}

	@Override
	public boolean isFinished() {
		// TODO Auto-generated method stub
		return this.climberPID.atGoal();
	}

	// TODO: We should add an end method that sets the motor output to 0 when the
	// command ends;
}
