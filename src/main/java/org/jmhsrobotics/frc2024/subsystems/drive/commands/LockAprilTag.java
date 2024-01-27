package org.jmhsrobotics.frc2024.subsystems.drive.commands;

import org.jmhsrobotics.frc2024.subsystems.drive.DriveSubsystem;
import org.jmhsrobotics.frc2024.subsystems.vision.VisionSubsystem;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

//TODO: move all hardcoded numbers to constants file
public class LockAprilTag extends Command {
	private DriveSubsystem drive;
	private VisionSubsystem vision;

	private ProfiledPIDController lockPID;
	private Constraints lockPIdConstraints;
	private Pose2d currentPose;
	private double currentYaw;

	private double fiducialID;

	private PhotonTrackedTarget target;

	private double angleGoal;

	public LockAprilTag(DriveSubsystem drive, VisionSubsystem vision, double fiducialID) {
		this.drive = drive;
		this.vision = vision;

		this.lockPIdConstraints = new Constraints(0.3, 5);
		this.lockPID = new ProfiledPIDController(0.3, 0, 0, this.lockPIdConstraints);

		this.fiducialID = fiducialID;
		this.target = this.vision.getTarget(this.fiducialID);

		this.angleGoal = this.target.getYaw();

		SmartDashboard.putData("LockPID", this.lockPID);
		addRequirements(this.drive, this.vision);
	}

	@Override
	public void initialize() {
		this.lockPID.reset(new State(this.angleGoal, 0));
		this.lockPID.setGoal(this.angleGoal);
		this.lockPID.setTolerance(1, 1);

		this.drive.stopDrive();
	}

	@Override
	public void execute() {
		if (this.vision.getTarget(this.fiducialID) != null) {
			this.currentPose = this.drive.getPose();
			this.currentYaw = this.drive.getPose().getRotation().getDegrees();

			this.lockPID.setConstraints(this.lockPIdConstraints);

			var rawOutput = this.lockPID.calculate(this.currentYaw);
			double output = MathUtil.clamp(rawOutput, -0.5, 0.5);

			this.drive.drive(0, 0, output, true, false);

			SmartDashboard.putNumber("LockPID/macAcc", this.lockPIdConstraints.maxAcceleration);
			SmartDashboard.putNumber("LockPID/maxVel", this.lockPIdConstraints.maxVelocity);
			SmartDashboard.putNumber("LockPID/PositionError", this.lockPID.getPositionError());
			SmartDashboard.putNumber("LockPID/VelocityError", this.lockPID.getVelocityError());
			SmartDashboard.putNumber("LockPID/output", output);
			SmartDashboard.putNumber("LockPID/currentYaw", currentYaw);
		}
	}

	@Override
	public boolean isFinished() {
		return this.lockPID.atGoal();
	}

	@Override
	public void end(boolean interrupted) {
		this.drive.drive(0, 0, 0, true, false);
	}
}
