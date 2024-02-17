package org.jmhsrobotics.frc2024.controlBoard;

import edu.wpi.first.wpilibj.XboxController;

public interface ControlBoard {

	// =============Operator Controls=============

	// =============Driver Controls=============

	// public double shooterInput();
	public boolean presetHigh();
	public boolean presetMid();
	public boolean presetLow();

	// =============Driver Controls=============
	public boolean brake();

	public boolean setZeroHeading();

	public double xInput();

	public double yInput();

	public double rotationalInput();

	public double pitchInput();

	public double intakeInput();

	public boolean shooterInput();

	public double extakeInput();

	public double climberExtend();

	public double climberRetract();

	public XboxController getDriverController();

	public XboxController getOperatorController();
}
