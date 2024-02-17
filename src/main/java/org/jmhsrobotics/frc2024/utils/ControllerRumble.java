package org.jmhsrobotics.frc2024.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

public class ControllerRumble extends Command {
	XboxController control;

	public ControllerRumble(XboxController control) {
		this.control = control;
	}

	public void setBothRumble(double value) {
		value = MathUtil.clamp(value, 0, 1);
		this.control.setRumble(RumbleType.kBothRumble, value);
	}

	public void setLefRumble(double value) {
		value = MathUtil.clamp(value, 0, 1);
		this.control.setRumble(RumbleType.kLeftRumble, value);
	}

	public void setRightRumble(double value) {
		value = MathUtil.clamp(value, 0, 1);
		this.control.setRumble(RumbleType.kRightRumble, value);
	}
}
