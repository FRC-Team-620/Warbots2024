package frc.robot.controlBoard;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ControlBoard {


//=============Operator Controls=============




    //=============Driver Controls=============
    public Trigger brake();
    public Trigger setZeroHeading();
    public double xInput();
    public double yInput();
    public double rotationalInput();

}
