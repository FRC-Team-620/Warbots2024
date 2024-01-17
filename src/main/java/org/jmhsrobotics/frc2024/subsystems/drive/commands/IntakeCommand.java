package org.jmhsrobotics.frc2024.subsystems.drive.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import org.jmhsrobotics.frc2024.subsystems.drive.DriveSubsystem;

import edu.wpi.first.wpilibj.Timer;

public class IntakeCommand extends Command {
    private DriveSubsystem drive;
    private double speed;
    private Timer timer;

    public IntakeCommand(DriveSubsystem drive, double speed) {
        this.drive = drive;
        this.speed = speed;
        this.timer = new Timer();
        addRequirements(this.drive);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        this.timer.reset();
        this.timer.start();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        SmartDashboard.putNumber("Intakecommand/intakespeed", this.speed);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return timer.hasElapsed(5);
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        SmartDashboard.putNumber("Intakecommand/intakespeed", Double.NaN);
        // TODO Testing to ensure I can make changes
    }
}