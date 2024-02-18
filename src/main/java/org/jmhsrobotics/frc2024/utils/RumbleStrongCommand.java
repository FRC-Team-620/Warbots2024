package org.jmhsrobotics.frc2024.utils;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

public class RumbleStrongCommand extends Command {

	/**
	 * A weak rumble on both sides of the controller
	 *
	 * @param controller
	 *            - the xbox controller that we're rumbling
	 */

	private XboxController controller;
	private RumbleType type;
	private double time, strength;

	private Timer timer;
	public RumbleStrongCommand(XboxController controller, RumbleType type, double time, double strength) {
		this.controller = controller;
		this.type = type;
		this.time = time;
		this.strength = strength;
	}

	@Override
	public void initialize() {
		controller.setRumble(this.type, 0);
		this.timer.reset();
	}

	@Override
	public void execute() {
		controller.setRumble(this.type, this.strength);
	}

	@Override
	public boolean isFinished() {
		return this.timer.hasElapsed(time);
	}

	@Override
	public void end(boolean interrupted) {
		controller.setRumble(RumbleType.kBothRumble, 0);
	}
}
