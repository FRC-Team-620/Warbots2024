package org.jmhsrobotics.frc2024.subsystems.climber.commands;

import org.jmhsrobotics.frc2024.subsystems.climber.ClimbSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimbCommand extends Command {
	private ClimbSubsystem climbSubsystem;
	private double lengthGoal;
	private ProfiledPIDController climbPID;
	private Constraints climberConstraints;

	public ClimbCommand(ClimbSubsystem climbSubsystem, double lengthGoal) {
		this.climbSubsystem = climbSubsystem;
		this.lengthGoal = lengthGoal;
		this.climberConstraints = new Constraints(4, 6);
		// TODO: tune PID vals
		this.climbPID = new ProfiledPIDController(.1, 0, 0, this.climberConstraints);

		addRequirements(climbSubsystem);
	}

	@Override
	public void initialize() {
		// TODO Auto-generated method stub
		this.climbPID.reset(this.climbSubsystem.getClimbPosition());
		this.climbPID.setGoal(lengthGoal);
		this.climbPID.setTolerance(1, 1);
	}

	@Override
	public void execute() {
		this.climbPID.setConstraints(climberConstraints);
		double motorOutput = this.climbPID.calculate(this.climbSubsystem.getClimbPosition());
		motorOutput = MathUtil.clamp(motorOutput, -1, 1);

		this.climbSubsystem.setMotor(motorOutput);

		SmartDashboard.putNumber("ClimberPID/positionError", this.climbPID.getPositionError());
		SmartDashboard.putNumber("ClimberPID/velocityError", this.climbPID.getVelocityError());
	}

	@Override
	public boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}

}
