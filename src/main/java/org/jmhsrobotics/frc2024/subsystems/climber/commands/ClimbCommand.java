package org.jmhsrobotics.frc2024.subsystems.climber.commands;

public class ClimbCommand {

<<<<<<< Updated upstream
=======
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimbCommand extends Command {
	ClimberSubsystem climberSubsystem;

	ProfiledPIDController rightClimberPID;
	Constraints rightClimberConstraint;

	ProfiledPIDController leftClimberPID;
	Constraints leftClimberConstraint;
	// in encodercount
	double positionGoal;

	public ClimbCommand(ClimberSubsystem climberSubsystem, double positionGoal) {
		this.climberSubsystem = climberSubsystem;
		this.positionGoal = positionGoal;

		this.rightClimberConstraint = new Constraints(80, 120);
		this.rightClimberPID = new ProfiledPIDController(0.01, 0, 0, this.rightClimberConstraint);

		this.leftClimberConstraint = new Constraints(80, 120);
		this.leftClimberPID = new ProfiledPIDController(0.01, 0, 0, this.leftClimberConstraint);
		SmartDashboard.putData("RightClimberPID", this.rightClimberPID);
		SmartDashboard.putData("LeftClimberPID", this.leftClimberPID);

		addRequirements(this.climberSubsystem);
	}

	@Override
	public void initialize() {
		this.rightClimberPID.reset(new State(this.positionGoal, 0));
		this.rightClimberPID.setGoal(this.positionGoal);
		this.rightClimberPID.setTolerance(0.5, 3);

		this.leftClimberPID.reset(new State(this.positionGoal, 0));
		this.leftClimberPID.setGoal(this.positionGoal);
		this.leftClimberPID.setTolerance(0.5, 3);
	}

	@Override
	public void execute() {
		// calculate motor output from pid controller

		this.rightClimberPID.setConstraints(new Constraints(80, 120));
		double rightMotorRawOutput = this.rightClimberPID.calculate(this.climberSubsystem.getRightEncoderPosition());
		double powerLim = 0.3;
		double rightMotorOutput = MathUtil.clamp(rightMotorRawOutput, -powerLim, powerLim);

		double leftMotorRawOutput = this.rightClimberPID.calculate(this.climberSubsystem.getRightEncoderPosition());
		double leftMotorOutput = MathUtil.clamp(leftMotorRawOutput, -powerLim, powerLim);

		// pass motor output to motor in subsystem
		this.climberSubsystem.setRightMotor(rightMotorOutput);
		this.climberSubsystem.setLeftMotor(leftMotorOutput);
		// SmartDashboard.putNumber("ExtensionPID/output", motorOutput);
		// SmartDashboard.putNumber("ExtensionPID/setpoint",
		// this.climberPID.getSetpoint().position); // 85 is max

		SmartDashboard.putNumber("ClimberPID/positionError", this.rightClimberPID.getPositionError());
		SmartDashboard.putNumber("ClimberPID/velocityError", this.rightClimberPID.getVelocityError());
	}

	@Override
	public boolean isFinished() {
		// TODO Auto-generated method stub
		return this.rightClimberPID.atGoal() || this.leftClimberPID.atGoal();
	}

	// TODO: We should add an end method that sets the motor output to 0 when the
	// command ends;
>>>>>>> Stashed changes
}
