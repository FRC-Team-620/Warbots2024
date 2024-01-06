package org.jmhsrobotics.subsystems.vision;

import org.jmhsrobotics.warcore.nt.NT4Util;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem
 extends SubsystemBase{
    PhotonCamera cam;
    public VisionSubsystem(){
        this.cam = new PhotonCamera("Arducam_OV9281_USB_Camera");
    }

    @Override
    public void periodic() {
        PhotonPipelineResult results = this.cam.getLatestResult();
        PhotonTrackedTarget target = results.getBestTarget();
        SmartDashboard.putBoolean("Vision/isConnected", this.cam.isConnected());
        if(target != null){
            Transform3d bestCameraToTarget = target.getBestCameraToTarget();
            SmartDashboard.putBoolean("Vision/hasTraget", results.hasTargets());
            SmartDashboard.putNumber("Vision/FiducialID", target.getFiducialId());
            NT4Util.putPose3d("Vision/target", new Pose3d(bestCameraToTarget.getTranslation(),bestCameraToTarget.getRotation()));
        }
    }
}
