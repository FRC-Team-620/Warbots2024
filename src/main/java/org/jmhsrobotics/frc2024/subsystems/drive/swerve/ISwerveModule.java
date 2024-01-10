package org.jmhsrobotics.frc2024.subsystems.drive.swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ISwerveModule {
    public SwerveModuleState getState();
    public SwerveModulePosition getPosition();
    public void setDesiredState(SwerveModuleState desiredState);
    public void resetEncoders();
    public void update(double dt); //TODO: Sim hack
}
