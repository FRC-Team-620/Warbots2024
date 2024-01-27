package org.jmhsrobotics.frc2024.subsystems.drive.commands;

import org.jmhsrobotics.frc2024.subsystems.drive.DriveSubsystem;
import org.jmhsrobotics.frc2024.subsystems.vision.VisionSubsystem;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

//TODO: move all hardcoded numbers to constants file
public class FollowAprilTag extends Command {
	private DriveSubsystem drive;
	private VisionSubsystem vision;

	private PIDController followPID;
	private double currentYaw;

	private double fiducialID;

	private double angleGoal;

	public FollowAprilTag(double fiducialID, DriveSubsystem drive, VisionSubsystem vision) {
		this.drive = drive;
		this.vision = vision;
		this.followPID = new PIDController(0.01, 0, 0);

		this.fiducialID = fiducialID;

		this.angleGoal = 0;

		// this.angleGoal = this.target.getYaw();

		SmartDashboard.putData("followPID", this.followPID);
		addRequirements(this.drive, this.vision);
	}

	@Override
	public void initialize() {
		this.followPID.setSetpoint(5);
		// our goal should be 0 degrees if the camera is in the center of the robot
		// Right now we are not accounting for the camera angle and cordnate sys

		this.drive.stopDrive();
	}

	@Override
	public void execute() {
		PhotonTrackedTarget aprilTag = this.vision.getTarget(this.fiducialID);
		if (aprilTag != null) {
			var rawOutput = this.followPID.calculate(aprilTag.getBestCameraToTarget().getX());
			double output = MathUtil.clamp(rawOutput, -0.1, 0.1);

			this.drive.drive(output, 0, 0, true, true);

			SmartDashboard.putNumber("followPID/PositionError", this.followPID.getPositionError());
			SmartDashboard.putNumber("followPID/VelocityError", this.followPID.getVelocityError());
			SmartDashboard.putNumber("followPID/output", output);
			SmartDashboard.putNumber("followPID/currentYaw", currentYaw);
		}
	}

	@Override
	public boolean isFinished() {
		return this.followPID.atSetpoint();
	}

	@Override
	public void end(boolean interrupted) {
		this.drive.drive(0, 0, 0, true, false);
	}
}
