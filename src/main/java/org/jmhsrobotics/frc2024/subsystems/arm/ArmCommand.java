package org.jmhsrobotics.frc2024.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class ArmCommand extends Command {

	private ArmSubsystem armSubsystem;
	private double angle;
	private ProfiledPIDController armPID;

	public ArmCommand(double angle, ArmSubsystem armSubsystem) {
		this.armSubsystem = armSubsystem;
		this.angle = angle;
		// TODO: tune Values;
		this.armPID = new ProfiledPIDController(1, 0, 0, new Constraints(5, 2.5));
		SmartDashboard.putData("ArmPID", this.armPID);
		addRequirements(armSubsystem);

	}
	@Override
	public void initialize() {
		// TODO Auto-generated method stub
		armPID.reset(new State(0, 0));
		armPID.setGoal(angle);
		armPID.setTolerance(1, 3);
	}

	@Override
	public void execute() {

		double PIDOut = armPID.calculate(armSubsystem.getArmPitch());
		PIDOut = MathUtil.clamp(PIDOut, -.9, .9);
		armSubsystem.setArmPivot(PIDOut);
		SmartDashboard.putNumber("ArmCommand/data/goal", this.angle);
		SmartDashboard.putNumber("ArmCommand/data/setPoint", this.armPID.getSetpoint().position);
		SmartDashboard.putNumber("ArmCommand/data/PIDOut", PIDOut);
		SmartDashboard.putNumber("ArmCommand/data/Current", this.armSubsystem.getArmPitch());
	}

	public boolean isFinished() {
		return armPID.atGoal();
	}

	public void end() {
		armSubsystem.setArmPivot(0);
	}
}
