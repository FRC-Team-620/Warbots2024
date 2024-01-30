package org.jmhsrobotics.frc2024.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;

import org.jmhsrobotics.warcore.math.TuneableProfiledPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmCommand extends Command {

	private ArmSubsystem armSubsystem;
	private double angle;
	private static ProfiledPIDController armPID;
	private static int num = 0;
	public ArmCommand(double angle, ArmSubsystem armSubsystem) {
		this.armSubsystem = armSubsystem;
		this.angle = angle;
		// TODO: tune Values;
		armPID = new TuneableProfiledPIDController(0.01, 0, 0, new Constraints(180, 180));
		SmartDashboard.putData("ArmPID2", armPID);
		addRequirements(armSubsystem);

	}

	@Override
	public void initialize() {
		// TODO Auto-generated method stub

		// armPID.reset(new State(0, 0));
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

	@Override
	public boolean isFinished() {
		return false;
		// return armPID.atGoal();
	}

	@Override
	public void end(boolean interrupted) {
		armSubsystem.setArmPivot(0);
	}
}
