package org.jmhsrobotics.frc2024.controlBoard;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CompControl implements ControlBoard {
	private XboxController driver = new XboxController(0);
	private XboxController operator = new XboxController(1);

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
	public Trigger brake() {
		return new JoystickButton(this.driver, XboxController.Button.kLeftBumper.value);
	}

	@Override
	public Trigger setZeroHeading() {
		return new JoystickButton(this.driver, XboxController.Button.kRightBumper.value);
	}

	// =============Operator Controls=============
	public double pitchInput() {
		return this.operator.getRightY();
	}

	public Trigger presetHigh() {
		return new JoystickButton(this.operator, XboxController.Button.kY.value);

	}

	@Override
	public Trigger presetMid() {
		return new JoystickButton(this.operator, XboxController.Button.kB.value);
	}

	public Trigger presetLow() {
		return new JoystickButton(this.operator, XboxController.Button.kA.value);
	}

	@Override
	public Trigger intakeInput() {
		// return new JoystickButton(this.operator,
		// XboxController.Axis.kRightTrigger.value);
		return new Trigger(this::intakeInputSupplier);
	}

	// idk wtf an EventLoop is to do this properly with operator.leftTrigger() but
	// this should work
	private boolean extakeInputSupplier() {
		return operator.getLeftTriggerAxis() > 0.5;
	}

	private boolean intakeInputSupplier() {
		return this.operator.getRightTriggerAxis() > 0.5;
	}

	@Override
	public Trigger extakeInput() {
		// return new JoystickButton(this.operator, );
		return new Trigger(this::extakeInputSupplier);

	}

	@Override
	public Trigger shooterInput() {
		return new JoystickButton(this.operator, XboxController.Button.kRightBumper.value);
	}

	@Override
	public Trigger climberExtend() {
		return new JoystickButton(this.operator, XboxController.Button.kStart.value);
	}

	@Override
	public Trigger climberRetract() {
		return new JoystickButton(this.operator, XboxController.Button.kBack.value);
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
