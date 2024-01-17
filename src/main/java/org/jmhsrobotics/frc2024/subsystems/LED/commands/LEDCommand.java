package org.jmhsrobotics.frc2024.subsystems.LED.commands;

import org.jmhsrobotics.frc2024.subsystems.LED.LEDSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class LEDCommand extends Command{
    
    private LEDSubsystem ledSubsystem;

    public LEDCommand(LEDSubsystem ledSubsystem){
        this.ledSubsystem = ledSubsystem;
    }

    @Override
    public void initialize() {
        this.ledSubsystem.startLED();
    }

    @Override
    public void execute() {
        this.ledSubsystem.setRGB(255, 0, 0);

        // create a rainbow effect
        // this.ledSubsystem.rainBow();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        this.ledSubsystem.endLED();
    }
}
