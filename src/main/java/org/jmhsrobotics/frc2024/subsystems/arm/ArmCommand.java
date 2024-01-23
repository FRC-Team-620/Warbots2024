package org.jmhsrobotics.frc2024.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
public class ArmCommand extends Command {

	private ArmSubsystem armSubsystem;
	private double angle;
	ProfiledPIDController armPID;

	public ArmCommand(double angle, ArmSubsystem armSubsystem) {
		this.armSubsystem = armSubsystem;
		this.angle = angle;
		// TODO: tune Values;
		this.armPID = new ProfiledPIDController(.1, 0, 0, null);

		addRequirements(armSubsystem);

	}
	@Override
	public void initialize() {
		// TODO Auto-generated method stub
		super.initialize();
		armPID.reset(new State(angle, 0));
		armPID.setGoal(angle);
		armPID.setTolerance(3, 3);
	}

	@Override
	public void execute() {

		double current = armPID.calculate(armSubsystem.getArmPitch());

		armSubsystem.setArmPivot(current);

	}

	public boolean isFinished() {
		return armPID.atGoal();
	}

	public void end() {
		armSubsystem.setArmPivot(0);
	}
}
