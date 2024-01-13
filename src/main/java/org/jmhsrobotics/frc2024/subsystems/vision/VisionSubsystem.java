package org.jmhsrobotics.frc2024.subsystems.vision;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.jmhsrobotics.frc2024.subsystems.drive.DriveSubsystem;
import org.jmhsrobotics.warcore.nt.NT4Util;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
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

		// Simulation
		simulationInit();
	}

	@Override
	public void periodic() {
		PhotonPipelineResult results = this.cam.getLatestResult();

		List<PhotonTrackedTarget> targets = results.getTargets();
		List<Pose3d> posList = new ArrayList<Pose3d>();
		List<Double> flucialIDs = new ArrayList<Double>();

		SmartDashboard.putBoolean("Vision/isConnected", this.cam.isConnected());

		for(PhotonTrackedTarget target : targets){
			Transform3d pos = target.getAlternateCameraToTarget();

			Pose3d pos3D = new Pose3d(pos.getTranslation(), pos.getRotation());
			posList.add(pos3D);
			flucialIDs.add(target.getFiducialId()+0.0);

		}
		SmartDashboard.putNumberArray("Vision/posList", (Double[])(flucialIDs.toArray()));
		NT4Util.putPose3d("Vision/poseList", (Pose3d[])(posList.toArray()));


		// if (target != null) {
		// 	Transform3d bestCameraToTarget = target.getBestCameraToTarget();
		// 	SmartDashboard.putBoolean("Vision/hasTraget", results.hasTargets());
		// 	SmartDashboard.putNumber("Vision/FiducialID", target.getFiducialId());
		// 	NT4Util.putPose3d("Vision/target",
		// 			new Pose3d(bestCameraToTarget.getTranslation(), bestCameraToTarget.getRotation()));
		// }

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


	VisionSystemSim visionSim = new VisionSystemSim("main");
	PhotonCameraSim cameraSim;

	private void simulationInit() {
		visionSim.addAprilTags(layout);
		SimCameraProperties cameraProp = new SimCameraProperties();
		cameraSim = new PhotonCameraSim(this.cam, cameraProp);
		Translation3d robotToCameraTrl = new Translation3d(0, 0, 0);
		// and pitched 15 degrees up.
		Rotation3d robotToCameraRot = new Rotation3d(0, 0, 0);
		Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

		// Add this camera to the vision system simulation with the given
		// robot-to-camera transform.
		visionSim.addCamera(cameraSim, robotToCamera);
		// A 640 x 480 camera with a 100 degree diagonal FOV.
		// cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
		// // Approximate detection noise with average and standard deviation error in
		// // pixels.
		// cameraProp.setCalibError(0.25, 0.08);
		// // Set the camera image capture framerate (Note: this is limited by robot
		// loop
		// // rate).
		// cameraProp.setFPS(20);
		// // The average and standard deviation in milliseconds of image data latency.
		// cameraProp.setAvgLatencyMs(35);
		// cameraProp.setLatencyStdDevMs(5);
		SmartDashboard.putData("VisionDebug", visionSim.getDebugField());
		cameraSim.enableRawStream(true);
		cameraSim.enableProcessedStream(true);

		// Enable drawing a wireframe visualization of the field to the camera streams.
		// This is extremely resource-intensive and is disabled by default.
		cameraSim.enableDrawWireframe(true);
	}

	@Override
	public void simulationPeriodic() {
		visionSim.update(drive.getPose());

	}
}
