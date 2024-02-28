package org.jmhsrobotics.frc2024.subsystems.arm.commands;

import org.jmhsrobotics.frc2024.subsystems.arm.ArmPIDSubsystem;
import org.jmhsrobotics.frc2024.subsystems.vision.VisionSubsystem;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;

//TODO: move all hardcoded numbers to constants file
public class ArmVision extends Command {
	private ArmPIDSubsystem arm;
	private VisionSubsystem vision;

	private Pose2d lastAprilTag;
	private InterpolatingDoubleTreeMap armAngles = new InterpolatingDoubleTreeMap();

	public ArmVision(ArmPIDSubsystem arm, VisionSubsystem vision) {
		this.arm = arm;
		this.vision = vision;
		armAngles.put(0d, 0d);
		armAngles.put(5d, 90d);

		addRequirements(this.arm, this.vision);
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		PhotonTrackedTarget aprilTag = this.vision.getTarget(7);
		if (aprilTag != null) {
			this.lastAprilTag = new Pose3d().transformBy(aprilTag.getBestCameraToTarget()).toPose2d(); // TODO: Clean up
																										// hack

		}
		if (this.lastAprilTag != null) {
			double dist = lastAprilTag.getTranslation().getDistance(new Translation2d());
			double angle = armAngles.get(dist);
			arm.setGoal(angle);
			// Transform2d transform = this.lastAprilTag.minus(this.arm.getPose());
			// double theta = Math.toDegrees(Math.atan2(transform.getY(),
			// transform.getX()));
			// // SmartDashboard.putNumber("Theta", theta);

			// var rawOutput = this.lockPID.calculate(theta);
			// double output = MathUtil.clamp(rawOutput, -0.5, 0.5);

			// // TODO: flip the output sign
			// this.arm.drive(0, 0, output, true, true);

			// SmartDashboard.putNumber("LockPID/PositionError",
			// this.lockPID.getPositionError());
			// SmartDashboard.putNumber("LockPID/VelocityError",
			// this.lockPID.getVelocityError());
			// SmartDashboard.putNumber("LockPID/output", output);
			// SmartDashboard.putNumber("LockPID/currentYaw", currentYaw);

		} else {
		}

	}

	@Override
	public boolean isFinished() {
		return false;
		// return false;
	}

	@Override
	public void end(boolean interrupted) {
		// this.arm.drive(0, 0, 0, true, false);
	}
}