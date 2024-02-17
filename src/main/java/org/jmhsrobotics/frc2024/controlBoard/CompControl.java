package org.jmhsrobotics.frc2024.controlBoard;

import edu.wpi.first.wpilibj.XboxController;

public class CompControl implements ControlBoard {
	private XboxController driver = new XboxController(0);
	private XboxController operator = new XboxController(1);

	// =============Driver Controls=============
	@Override
	public double xInput() {

		return this.driver.getLeftX();
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
		return this.driver.getRightBumper();
	}

	@Override
	public boolean setZeroHeading() {
		return this.driver.getLeftBumper();
	}

	// =============Operator Controls=============
	public double pitchInput() {
		return this.operator.getRightY();
	}

	public boolean presetHigh() {
		return this.operator.getAButton();
	}

	@Override
	public boolean presetMid() {
		return this.operator.getAButton();
	}

	public boolean presetLow() {
		return this.operator.getAButton();
	}

	@Override
	public double intakeInput() {
		return this.operator.getRightTriggerAxis();
	}

	@Override
	public double extakeInput() {
		return this.operator.getLeftTriggerAxis();
	}

	@Override
	public boolean shooterInput() {
		return this.operator.getRightBumper();
	}

	@Override
	public double climberExtend() {
		return this.operator.getRightTriggerAxis();
	}

	@Override
	public double climberRetract() {
		return this.operator.getRightTriggerAxis();
	}
}
