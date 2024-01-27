package org.jmhsrobotics.frc2024.subsystems.drive.commands;

import org.jmhsrobotics.frc2024.subsystems.drive.DriveSubsystem;
import org.jmhsrobotics.frc2024.subsystems.vision.VisionSubsystem;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class FollowAprilTagCommand extends Command {
	private DriveSubsystem drive;
	private VisionSubsystem vision;

	private PIDController thetaPID;
	private PIDController xPID;
	private PIDController yPID;

	private double fiducialID;

	private double xGoal;
	private double yGoal;
	private double thetaGoal;
	private Field2d testing = new Field2d();

	public FollowAprilTagCommand(double fiducialID, DriveSubsystem drive, VisionSubsystem vision) {
		this.drive = drive;
		this.vision = vision;

		this.xPID = new PIDController(0.01, 0, 0);
		this.yPID = new PIDController(0.01, 0, 0);
		this.thetaPID = new PIDController(0.01, 0, 0);

		this.fiducialID = fiducialID;

		this.thetaGoal = 0;
		this.xGoal = 2;
		this.yGoal = -2;
		// this.angleGoal = this.target.getYaw();
		testing.getObject("target").setPose(new Pose2d(3, 3, new Rotation2d()));
		SmartDashboard.putData("LockPID", this.thetaPID);
		SmartDashboard.putData("testingField", testing);
		addRequirements(this.drive, this.vision);
	}

	@Override
	public void initialize() {
		this.xPID.reset();
		this.xPID.setSetpoint(this.xGoal);
		this.xPID.setTolerance(3, 1);

		this.yPID.reset();
		this.yPID.setSetpoint(this.yGoal);
		this.yPID.setTolerance(3, 1);

		// this.xPID.setSetpoint();
		this.thetaPID.reset();
		this.thetaPID.setSetpoint(this.thetaGoal);
		this.thetaPID.setTolerance(3, 1);
		// our goal should be 0 degrees if the camera is in the center of the robot
		// Right now we are not accounting for the camera angle and cordnate sys

		this.drive.stopDrive();
	}

	@Override
	public void execute() {
		testing.setRobotPose(drive.getPose());
		PhotonTrackedTarget aprilTag = this.vision.getTarget(this.fiducialID);

		// Transform3d theta = aprilTag.getBestCameraToTarget();
		// double x = aprilTag.getBestCameraToTarget().getX();
		// double x =
		// testing.getObject("target").getPose().minus(this.drive.getPose()).getX();
		Transform2d transform = this.drive.getPose().minus(testing.getObject("target").getPose());
		double x = transform.getX();
		SmartDashboard.putNumber("FollowAprilTag/X", x);

		double y = transform.getY();
		SmartDashboard.putNumber("FollowAprilTag/Y", y);

		double theta = transform.getRotation().getDegrees();
		SmartDashboard.putNumber("FollowAprilTag/theta", theta);

		// double y = aprilTag.getBestCameraToTarget().getY();

		// if (aprilTag != null) {
		// var rawXOutput = this.xPID.calculate(x);
		// double xOutput = MathUtil.clamp(rawXOutput, 0.2, 0.2);

		// this.drive.drive(xOutput, 0, 0, false, true);

		// SmartDashboard.putNumber("LockPID/PositionError",
		// this.thetaPID.getPositionError());
		// SmartDashboard.putNumber("LockPID/VelocityError",
		// this.thetaPID.getVelocityError());
		// // SmartDashboard.putNumber("LockPID/output", thetaOutput);
		// } else {
		// this.drive.drive(0, 0, 0, false, true);
		// }

		var rawXOutput = this.xPID.calculate(x);
		double xOutput = MathUtil.clamp(rawXOutput, -0.2, 0.2);

		var rawYOutput = this.yPID.calculate(y);
		double yOutPut = MathUtil.clamp(rawYOutput, -0.2, 0.2);

		var rawThetaOutput = this.thetaPID.calculate(theta);
		double thetaOutput = MathUtil.clamp(rawThetaOutput, -0.4, 0.4);

		this.drive.drive(xOutput, yOutPut, thetaOutput, false, true);
		SmartDashboard.putNumber("LockPID/PositionError", this.thetaPID.getPositionError());
		SmartDashboard.putNumber("LockPID/VelocityError", this.thetaPID.getVelocityError());
	}

	@Override
	public boolean isFinished() {
		// return this.thetaPID.atSetpoint();
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		this.drive.drive(0, 0, 0, true, false);
	}
}
