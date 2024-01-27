package org.jmhsrobotics.frc2024.subsystems.drive.commands;

import org.jmhsrobotics.frc2024.subsystems.drive.DriveSubsystem;
import org.jmhsrobotics.frc2024.subsystems.vision.VisionSubsystem;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

//TODO: move all hardcoded numbers to constants file
public class LockAprilTag extends Command {
	private DriveSubsystem drive;
	private VisionSubsystem vision;

	private PIDController lockPID;
	private double currentYaw;

	private double fiducialID;

	private double angleGoal;

	public LockAprilTag(double fiducialID, DriveSubsystem drive, VisionSubsystem vision) {
		this.drive = drive;
		this.vision = vision;
		this.lockPID = new PIDController(0.01, 0, 0);

		this.fiducialID = fiducialID;

		this.angleGoal = 0;

		// this.angleGoal = this.target.getYaw();

		SmartDashboard.putData("LockPID", this.lockPID);
		addRequirements(this.drive, this.vision);
	}

	@Override
	public void initialize() {
		this.lockPID.reset();
		this.lockPID.setSetpoint(this.angleGoal);
		this.lockPID.setTolerance(3, 1);
		// our goal should be 0 degrees if the camera is in the center of the robot
		// Right now we are not accounting for the camera angle and cordnate sys

		this.drive.stopDrive();
	}

	@Override
	public void execute() {
		PhotonTrackedTarget aprilTag = this.vision.getTarget(this.fiducialID);
		if (aprilTag != null) {
			var rawOutput = this.lockPID.calculate(aprilTag.getYaw());
			double output = MathUtil.clamp(rawOutput, -0.5, 0.5);

			this.drive.drive(0, 0, output, true, true);

			SmartDashboard.putNumber("LockPID/PositionError", this.lockPID.getPositionError());
			SmartDashboard.putNumber("LockPID/VelocityError", this.lockPID.getVelocityError());
			SmartDashboard.putNumber("LockPID/output", output);
			SmartDashboard.putNumber("LockPID/currentYaw", currentYaw);
		} else {
			this.drive.drive(0, 0, 0, true, true);
		}
	}

	@Override
	public boolean isFinished() {
		return this.lockPID.atSetpoint();
		// return false;
	}

	@Override
	public void end(boolean interrupted) {
		this.drive.drive(0, 0, 0, true, false);
	}
}
