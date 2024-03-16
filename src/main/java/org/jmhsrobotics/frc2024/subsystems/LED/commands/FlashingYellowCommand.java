package org.jmhsrobotics.frc2024.subsystems.LED.commands;

import org.jmhsrobotics.frc2024.subsystems.LED.LEDSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class FlashingYellowCommand extends Command{
    
    private LEDSubsystem ledSubsystem;
    private boolean isFlahing = true;
    public FlashingYellowCommand(LEDSubsystem led){
        this.ledSubsystem = led;
    }

    @Override
	public void initialize() {
		this.ledSubsystem.startLED();

	}

	@Override
	public void execute() {
        if(this.isFlahing){
            this.ledSubsystem.setRGB(255,255,224);
            this.isFlahing = false;
            this.ledSubsystem.endLED();
        }else{
            
        }
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
}
