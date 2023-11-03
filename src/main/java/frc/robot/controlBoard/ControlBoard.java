package frc.robot.controlBoard;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ControlBoard {
    public Trigger brake();
    public Trigger zeroHeading();


}
