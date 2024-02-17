package org.jmhsrobotics.frc2024.controlBoard;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ControlBoard {

	// =============Operator Controls=============

    // =============Driver Controls=============

	// public double shooterInput();
	public Trigger presetHigh();
	public Trigger presetMid();
	public Trigger presetLow();

	// =============Driver Controls=============
	public Trigger brake();

	public Trigger setZeroHeading();

	public double xInput();

	public double yInput();

	public double rotationalInput();

	public double pitchInput();

	public Trigger intakeInput();

	public Trigger shooterInput();

	public Trigger extakeInput();

	public Trigger climberExtend();

	public Trigger climberRetract();
}
