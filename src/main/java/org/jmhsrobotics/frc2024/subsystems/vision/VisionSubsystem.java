package org.jmhsrobotics.frc2024.subsystems.vision;

import java.io.IOException;
import java.util.Optional;

import org.jmhsrobotics.frc2024.subsystems.drive.DriveSubsystem;
import org.jmhsrobotics.warcore.nt.NT4Util;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
	// load the apriltag field layout
	AprilTagFieldLayout layout;

	// declare the camera
	PhotonCamera cam;

	// get the camera position on the robot
	Transform3d camOnRobot = new Transform3d(new Translation3d(0.5, 0, 0.3), new Rotation3d());

	// construct a photonPoseEstimator
	PhotonPoseEstimator estimator;

	DriveSubsystem drive;

	public VisionSubsystem(DriveSubsystem drive) {
		this.cam = new PhotonCamera("clarance");
		this.estimator = new PhotonPoseEstimator(this.layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, this.cam,
				this.camOnRobot);
		this.drive = drive;

		System.out.println(Filesystem.getDeployDirectory().getAbsolutePath());
		try {
			layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
		} catch (IOException e) {
			// TODO: error handling
			System.out.println(e);
			DriverStation.reportError("Fail to load the april tag map", e.getStackTrace());
		}
		estimator.setFieldTags(layout);
	}

	@Override
	public void periodic() {
		PhotonPipelineResult results = this.cam.getLatestResult();
		PhotonTrackedTarget target = results.getBestTarget();
		SmartDashboard.putBoolean("Vision/isConnected", this.cam.isConnected());
		if (target != null) {
			Transform3d bestCameraToTarget = target.getBestCameraToTarget();
			SmartDashboard.putBoolean("Vision/hasTraget", results.hasTargets());
			SmartDashboard.putNumber("Vision/FiducialID", target.getFiducialId());
			NT4Util.putPose3d("Vision/target",
					new Pose3d(bestCameraToTarget.getTranslation(), bestCameraToTarget.getRotation()));
		}

		// Puting the estimated pose to the network table
		var estimatedPose = this.getEstimatedGlobalPose(this.drive.getPose());
		if (estimatedPose.isPresent()) {
			NT4Util.putPose3d("Vision/EstimatedTarget", estimatedPose.get().estimatedPose);
		} else {
			NT4Util.putPose3d("Vision/EstimatedTarget", new Pose3d());
		}
	}

	public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevPose) {
		this.estimator.setReferencePose(prevPose);
		return this.estimator.update();
	}
}
