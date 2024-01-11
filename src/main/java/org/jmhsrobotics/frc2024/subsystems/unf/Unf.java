package org.jmhsrobotics.frc2024.subsystems.unf;

import org.jmhsrobotics.frc2024.subsystems.drive.DriveConstants.SwerveConstants;
import org.jmhsrobotics.frc2024.subsystems.drive.swerve.ISwerveModule;
import org.jmhsrobotics.frc2024.subsystems.drive.swerve.SimSwerveModule;
import org.jmhsrobotics.warcore.nt.NT4Util;
import org.jmhsrobotics.warcore.swerve.SwerveVisualizer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Unf extends SubsystemBase {

    private SwerveVisualizer m_visualizer;
    JankSwerveOdom odom;
    SwerveDriveKinematics kinematics;
    private ISwerveModule m_frontLeft, m_frontRight, m_rearLeft, m_rearRight;
    Translation2d fl = new Translation2d(SwerveConstants.kWheelBase / 2, SwerveConstants.kTrackWidth / 2);
    Translation2d fr =new Translation2d(SwerveConstants.kWheelBase / 2, -SwerveConstants.kTrackWidth / 2);
    Translation2d rl =new Translation2d(-SwerveConstants.kWheelBase / 2, SwerveConstants.kTrackWidth / 2);
    Translation2d rr = new Translation2d(-SwerveConstants.kWheelBase / 2, -SwerveConstants.kTrackWidth / 2);
    public Rotation2d gyro = new Rotation2d();
    public Field2d f = new Field2d();
    public Unf() {
        SmartDashboard.putData(f);
        
        m_frontLeft = new SimSwerveModule();
        m_frontRight = new SimSwerveModule();
        m_rearLeft = new SimSwerveModule();
        m_rearRight = new SimSwerveModule();
        
        // kinematics = new SwerveDriveKinematics(new Translation2d(1, 1), new Translation2d(-1, 1),
        //         new Translation2d(-1, -1), new Translation2d(1, -1));
        kinematics = SwerveConstants.kDriveKinematics;
        // kinematics = new SwerveDriveKinematics(fl,rl,rr,fr);
        odom = new JankSwerveOdom(kinematics, gyro, new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
        });

        m_visualizer = new SwerveVisualizer(2, 2);
    }

    @Override
    public void periodic() {
       

        // kinematics.m_visualizer.update();
    }
    public Rotation2d desRot = new Rotation2d();
    public Pose2d pose = new Pose2d();
    public void drive(double x, double y, double theta) {
        desRot = Rotation2d.fromRadians(theta);
        var states = kinematics.toSwerveModuleStates(new ChassisSpeeds(x, y, theta));
        m_frontLeft.setDesiredState(states[0]);
        m_frontRight.setDesiredState(states[1]);
        m_rearLeft.setDesiredState(states[2]);
        m_rearRight.setDesiredState(states[3]);
        
         NT4Util.putPose2d("fl", new Pose2d(fl,m_frontLeft.getPosition().angle));
        NT4Util.putPose2d("fr", new Pose2d(fr,m_frontRight.getPosition().angle));
        NT4Util.putPose2d("rl", new Pose2d(rl,m_rearLeft.getPosition().angle));
        NT4Util.putPose2d("rr", new Pose2d(rr,m_rearRight.getPosition().angle));
        pose = odom.update(gyro,new SwerveModulePosition[]{m_frontLeft.getPosition(), m_frontRight.getPosition(), m_rearLeft.getPosition(), m_rearRight.getPosition()});
        m_visualizer.update(m_frontLeft.getState().angle, m_frontRight.getState().angle, m_rearLeft.getState().angle, m_rearRight.getState().angle, pose);

    }
    @Override
    public void simulationPeriodic() {
        m_frontLeft.update(0.2);
        m_frontRight.update(0.2);
        m_rearLeft.update(0.2);
        m_rearRight.update(0.2);
        // gyro = gyro.plus(desRot.times(0.02));

    }
//     @Override
//   public void simulationPeriodic() {
//     var deltapos = new SwerveModulePosition[] {
//         getPoseDeltas(m_frontLeft),
//         getPoseDeltas(m_frontRight),
//         getPoseDeltas(m_rearLeft),
//         getPoseDeltas(m_rearRight)
//     };

//     Twist2d twist = SwerveConstants.kDriveKinematics.toTwist2d(deltapos);
//     // pos = pos.exp(twist);
//     SmartDashboard.putNumber("dtheta", twist.dtheta);
//     // imuSim.addYaw(Math.toDegrees(twist.dtheta));
//     gyro = gyro.plus(Rotation2d.fromRadians(twist.dtheta));

//   }

  private SwerveModulePosition getPoseDeltas(ISwerveModule module) {
    // var start = module.getPosition().copy();
    var startpos = module.getPosition().distanceMeters;
    var startdeg = module.getPosition().angle.getDegrees();
    module.update(SwerveConstants.dtOffset);
    // var end = module.getPosition();
    var endpos = module.getPosition().distanceMeters;
    var enddeg = module.getPosition().angle.getDegrees();

    return new SwerveModulePosition( endpos- startpos, new Rotation2d(enddeg-startdeg));
  }

}
