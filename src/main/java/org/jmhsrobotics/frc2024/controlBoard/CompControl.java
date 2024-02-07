package org.jmhsrobotics.frc2024.controlBoard;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CompControl implements ControlBoard {
	private CommandXboxController driver = new CommandXboxController(0);
	private CommandXboxController operator = new CommandXboxController(1);

	// TODO: Implement operator controls in the future
	// =============Operator Controls=============
	// public double shooterInput() {
	// return this.operator.getRightTriggerAxis();
	// }

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

	public double pitchInput() {
		return this.driver.getRightY();
	}

	public Trigger presetHigh() {
		return this.driver.y();
	}

	public Trigger presetLow() {
		return this.driver.a();
	}

	@Override
	public Trigger brake() {
		return this.driver.leftBumper();
	}

	@Override
	public Trigger setZeroHeading() {
		return this.driver.rightBumper();
	}

	@Override
	public Trigger intakeInput() {
		return this.operator.leftTrigger();
	}

	@Override
	public Trigger shooterInput() {
		return this.operator.rightTrigger();
	}

	@Override
	public Trigger extakeInput() {
		return this.operator.povDown();
	}

}
