package org.jmhsrobotics.frc2024.utils.newcmd;

import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem;
import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem.ControlType;

import edu.wpi.first.wpilibj2.command.Command;

public class NSpinupNoStop extends Command {
    private ShooterSubsystem shooter;
    private double targetRPM;
    public NSpinupNoStop(ShooterSubsystem shooter, double rpm){
        this.shooter = shooter;
        this.targetRPM = rpm;
        addRequirements(shooter);
    }
    @Override
    public void initialize() {
        shooter.set(targetRPM, ControlType.PID);
    }
    @Override
    public boolean isFinished() {
        return false;
    }

    //TODO: No end method to help prevent hickups?

    
}
