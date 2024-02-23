package org.jmhsrobotics.frc2024.subsystems.shooter.commands;

import org.jmhsrobotics.frc2024.subsystems.shooter.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterAutoCommand extends Command{
    private ShooterSubsystem shooterSubsystem;
    private double targetRPM;

    public ShooterAutoCommand(ShooterAutoCommand shooterAutoCommand, double targetRPM){
        this.shooterSubsystem = shooterSubsystem;
        this.targetRPM = targetRPM;
    }
    @Override
    public void execute() {
        this.shooterSubsystem.setGoal(this.targetRPM);
    }

    @Override
    public boolean isFinished() {
        return this.shooterSubsystem.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        this.shooterSubsystem.setVolt(0);
    }
}
