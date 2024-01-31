package org.jmhsrobotics.frc2024.controlBoard;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ControlBoard {

    // =============Operator Controls=============

    // =============Driver Controls=============
    public boolean brake();

    public boolean setZeroHeading();

    public double xInput();

    public double yInput();

    public double rotationalInput();

    public XboxController getDriverController();

    public XboxController getOperatorController();
}
