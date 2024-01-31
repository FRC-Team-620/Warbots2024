package org.jmhsrobotics.frc2024.controlBoard;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CompControl implements ControlBoard {
    private XboxController driver = new XboxController(0);
    private XboxController operator = new XboxController(1);

    // TODO: Implement operator controls in the future
    // =============Operator Controls=============

    // =============Driver Controls=============
    @Override
    public double xInput() {

        return this.driver.getLeftX();
    }

    @Override
    public double yInput() {
        return this.driver.getLeftY();
    }

    @Override
    public double rotationalInput() {
        return this.driver.getRightX();
    }

    @Override
    public boolean brake() {
        return this.driver.getLeftBumper();
    }

    @Override
    public boolean setZeroHeading() {
        return this.driver.getRightBumper();
    }

    // =============Utils=============
    @Override
    public XboxController getDriverController() {
        return this.driver;
    }

    @Override
    public XboxController getOperatorController() {
        return this.operator;
    }

}
