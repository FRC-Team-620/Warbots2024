package org.jmhsrobotics.frc2024.subsystems.drive.commands;

import org.jmhsrobotics.frc2024.subsystems.drive.DriveSubsystem;
import org.jmhsrobotics.frc2024.subsystems.vision.VisionSubsystem;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;

public class LockAprilTag extends Command {
	private DriveSubsystem drive;
	private VisionSubsystem vision;

	private ProfiledPIDController lockPID;
	private Constraints lockPIdConstraints;
	private Pose2d currentPose;
	private double currentAngle;

	private double fiducialID;

	public LockAprilTag(DriveSubsystem drive, VisionSubsystem vision, double fiducialID) {
		this.drive = drive;
		this.vision = vision;

		this.lockPIdConstraints = new Constraints(0.3, 5);
		this.lockPID = new ProfiledPIDController(0.3, 0, 0, this.lockPIdConstraints);

		this.fiducialID = fiducialID;
		addRequirements(this.drive, this.vision);
	}

	@Override
	public void initialize() {
		this.lockPID.reset(new State(0, 0));
		this.lockPID.setGoal(0);
	}

	@Override
	public void execute() {
		this.currentPose = this.drive.getPose();
		this.currentAngle = this.drive.getPose().getRotation().getDegrees();

	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		this.drive.drive(0, 0, 0, true, false);
	}
}
