package org.jmhsrobotics.frc2024.subsystems.vision;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.jmhsrobotics.frc2024.Robot;
import org.jmhsrobotics.frc2024.subsystems.drive.DriveSubsystem;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;

public class VisionSubsystem extends SubsystemBase implements Logged {

	// load the apriltag field layout
	private AprilTagFieldLayout layout;

	// declare the camera
	private PhotonCamera cam;

	// get the camera position on the robot
	private Transform3d camOnRobot = new Transform3d(
			new Translation3d(Units.inchesToMeters(-13), Units.inchesToMeters(0), Units.inchesToMeters(9.875)),
			new Rotation3d(0, Units.degreesToRadians(-20), (Math.PI)));

	// construct a photonPoseEstimator
	private PhotonPoseEstimator estimator;

	// declare driveSubsystem
	private DriveSubsystem drive;

	// declare fiducialIDs (apriltag names)
	private double[] fiducialIDs;

	// declare detected apriltags
	List<PhotonTrackedTarget> targets;

	public VisionSubsystem(DriveSubsystem drive) {

		// instantiate camera
		this.cam = new PhotonCamera("Sechenov");

		// instantiate drivetrain subsystem
		this.drive = drive;

		// load and configure apriltag locations
		// this.loadAprilTagLocations();

		// Simulation
		if (Robot.isSimulation()) {
			simulationInit();
		}
	}

	private void loadAprilTagLocations() {
		// instantiate global position pose estimator
		this.estimator = new PhotonPoseEstimator(this.layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, this.cam,
				this.camOnRobot);

		// load apriltag locations from JSON
		try {
			layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
		} catch (IOException e) {
			System.out.println(e);
			DriverStation.reportError("Fail to load the april tag map", e.getStackTrace());
		}

		// set the apriltag locations to the photonPoseEstimator
		estimator.setFieldTags(layout);
	}

	@Override
	public void periodic() {

		/*
		 * TODO: check if previous result timestamp is equal to current result timestamp
		 * and only load if different
		 */

		// get the latest camera results (camera and apriltag properties)
		PhotonPipelineResult results = this.cam.getLatestResult();

		// extract detected apriltags
		targets = results.getTargets();

		// get number of detected targets
		int len = targets.size();

		// declare fieldcentric tag pose list
		Pose3d[] posList = new Pose3d[len];

		// declare list of apriltag fiducial IDs
		fiducialIDs = new double[len];

		// for every target
		for (int i = 0; i < len; i++) {
			// Get robot position relative to field
			Pose3d robotPose3d = new Pose3d(this.drive.getPose());

			// Get tag transform relative to camera
			Transform3d targetTransFromCam = targets.get(i).getBestCameraToTarget();

			// Calculate field relative position of apriltag
			Pose3d outPutPose = robotPose3d.plus(camOnRobot.plus(targetTransFromCam));

			// Add apriltag position to list of fieldcentric poses
			posList[i] = outPutPose;
			fiducialIDs[i] = targets.get(i).getFiducialId();
		}

		log("fiducialIDs", fiducialIDs);
		log("targets", posList);
		log("cameraTransform", camOnRobot); // TODO: only log once

		// Puting the estimated pose to the network table
		// this.putEstimatedPoseToNT();
	}

	private void putEstimatedPoseToNT() {
		var estimatedPose = this.getEstimatedGlobalPose(this.drive.getPose());
		if (estimatedPose.isPresent()) {
			// NT4Util.putPose3d("Vision/EstimatedTarget",
			// estimatedPose.get().estimatedPose);
			log("postEstimation", estimatedPose.get().estimatedPose);
		}
	}

	public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevPose) {
		this.estimator.setReferencePose(prevPose);
		return this.estimator.update();
	}

	public PhotonTrackedTarget getTarget(double fiducialID) {
		for (PhotonTrackedTarget i : targets) {
			if (i.getFiducialId() == fiducialID) {
				return i;
			}
		}
		return null;
	}

	public AprilTagFieldLayout getAprilTagLayout() {
		return this.layout;
	}

	public Transform3d targetToCamera(Transform3d target) {
		return camOnRobot.plus(target);
	}

	public Pose3d targetToField(Transform3d target, Pose2d robotPose) {
		return new Pose3d(robotPose).plus(camOnRobot.plus(target));
	}

	VisionSystemSim visionSim = new VisionSystemSim("main");
	PhotonCameraSim cameraSim;

	private void simulationInit() {
		if (layout != null) {
			visionSim.addAprilTags(layout);
		}
		SimCameraProperties cameraProp = new SimCameraProperties();
		cameraSim = new PhotonCameraSim(this.cam, cameraProp);
		// robot-to-camera transform.
		visionSim.addCamera(cameraSim, camOnRobot);
		// A 640 x 480 camera with a 100 degree diagonal FOV.
		// cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
		// // Approximate detection noise with average and standard deviation error in
		// // pixels.
		// cameraProp.setCalibError(0.25, 0.08);
		// // Set the camera image capture framerate (Note: this is limited by robot
		// loop
		// // rate).
		cameraProp.setFPS(10);
		// // The average and standard deviation in milliseconds of image data latency.
		cameraProp.setAvgLatencyMs(50);
		cameraProp.setLatencyStdDevMs(5);
		// SmartDashboard.putData("VisionDebug", visionSim.getDebugField());
		// cameraSim.enableRawStream(true);
		// cameraSim.enableProcessedStream(true);

		// // Enable drawing a wireframe visualization of the field to the camera
		// streams.
		// // This is extremely resource-intensive and is disabled by default.
		// cameraSim.enableDrawWireframe(true);
	}

	@Override
	public void simulationPeriodic() {
		visionSim.update(drive.simpos); // Slight hack but fixes latency issue

	}
}
