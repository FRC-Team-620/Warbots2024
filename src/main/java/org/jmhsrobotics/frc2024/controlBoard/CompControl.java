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
		// return new JoystickButton(this.operator, XboxController.Axis.kRightTrigger.value);
		return new Trigger(this.operator.rightTrigger(null));
	}

	@Override
	public Trigger extakeInput() {
		return new JoystickButton(this.operator, XboxController.Axis.kLeftTrigger.value);
	}

	@Override
	public Trigger shooterInput() {
		return new JoystickButton(this.operator, XboxController.Button.kRightBumper.value);
	}

	@Override
	public Trigger climberExtend() {
		return new JoystickButton(this.operator, XboxController.Axis.kRightY.value);
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
