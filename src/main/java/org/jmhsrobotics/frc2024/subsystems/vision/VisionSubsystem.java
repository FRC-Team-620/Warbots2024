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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem
        extends SubsystemBase {
    // load the apriltag field layout
    AprilTagFieldLayout layout;

    //declare the camera
    PhotonCamera cam;
    
    // get the camera position on the robot
    Transform3d camOnRobot = new Transform3d(new Translation3d(0.5, 0, 0.3), new Rotation3d());

    //construct a photonPoseEstimator
    PhotonPoseEstimator estimator = new PhotonPoseEstimator(this.layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, this.camOnRobot);

    DriveSubsystem drive;

    public VisionSubsystem() {
        this.cam = new PhotonCamera("Arducam_OV9281_USB_Camera");
        this.drive = new DriveSubsystem();
        try{
            AprilTagFieldLayout.loadFromResource(Filesystem.getDeployDirectory().toString() + "2024-crescendo.json");
        }
        catch(IOException e){
            //TODO: error handling
        }
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

        //Puting the estimated pose to the network table
        NT4Util.putPose3d("Vision/EstimatedTarget",this.getEstimatedGlobalPose(this.drive.getPose()).get().estimatedPose);
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevPose){
        this.estimator.setReferencePose(prevPose);
        return this.estimator.update();
    }
}
