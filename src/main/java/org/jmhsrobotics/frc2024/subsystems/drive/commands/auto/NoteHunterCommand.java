package org.jmhsrobotics.frc2024.subsystems.drive.commands.auto;

import org.jmhsrobotics.frc2024.subsystems.drive.DriveSubsystem;
import org.jmhsrobotics.frc2024.subsystems.vision.VisionSubsystem;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class NoteHunterCommand extends Command {
	private DriveSubsystem drive;
	private VisionSubsystem vision;

	private PIDController thetaPID;
	private PIDController xPID;
	private PIDController yPID;

	private double xGoal;
	private double yGoal;
	private double thetaGoal;

	public NoteHunterCommand(VisionSubsystem vision, DriveSubsystem drive) {
		this.drive = drive;
		this.vision = vision;

		this.xPID = new PIDController(0.1, 0, 0);
		this.yPID = new PIDController(0.1, 0, 0);
		this.thetaPID = new PIDController(0.0125, 0, 0.0001);

		this.xGoal = 1;
		this.yGoal = 1;
		this.thetaGoal = 175;

		SmartDashboard.putData("xPID", this.xPID);
		SmartDashboard.putData("yPID", this.yPID);
		SmartDashboard.putData("thetaPID", this.thetaPID);
		addRequirements(this.drive);
	}
	@Override
	public void initialize() {
		this.xPID.setSetpoint(this.xGoal);
		this.xPID.setTolerance(3, 1);

		this.yPID.reset();
		this.yPID.setSetpoint(this.yGoal);
		this.yPID.setTolerance(3, 1);

		// this.xPID.setSetpoint();
		this.thetaPID.reset();
		this.thetaPID.setSetpoint(this.thetaGoal);
		this.thetaPID.setTolerance(3, 1);
		this.thetaPID.enableContinuousInput(-180, 180);
		// our goal should be 0 degrees if the camera is in the center of the robot
		// Right now we are not accounting for the camera angle and cordnate sys

		this.drive.stopDrive();
	}

	@Override
	public void execute() {
		PhotonTrackedTarget piece = this.vision.getBestObjectTarget();
		Transform2d trans = new Transform2d(piece.getBestCameraToTarget().getTranslation().toTranslation2d(),
				piece.getBestCameraToTarget().getRotation().toRotation2d());
		double x = trans.getX();
		double y = trans.getY();
		double theta = trans.getRotation().getDegrees();
		var rawXOutput = this.xPID.calculate(x);
		double xOutput = MathUtil.clamp(rawXOutput, -0.3, 0.3);

		var rawYOutput = this.yPID.calculate(y);
		double yOutPut = MathUtil.clamp(rawYOutput, -0.3, 0.3);

		var rawThetaOutput = this.thetaPID.calculate(theta);
		double thetaOutput = MathUtil.clamp(rawThetaOutput, -0.1, 0.1);

		this.drive.drive(-xOutput, -yOutPut, -thetaOutput, false, true);
	}

	@Override
	public boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		this.drive.drive(0, 0, 0, true, false);
	}
}
