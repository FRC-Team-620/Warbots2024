package org.jmhsrobotics.frc2024.utils;

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
	public RumbleStrongCommand(XboxController controller) {
		this.controller = controller;
	}

	@Override
	public void initialize() {
		controller.setRumble(RumbleType.kBothRumble, 0);
	}

	@Override
	public void execute() {
		controller.setRumble(RumbleType.kBothRumble, 0.5);
	}

	@Override
	public boolean isFinished() {

		return false;
	}

	@Override
	public void end(boolean interrupted) {
		controller.setRumble(RumbleType.kBothRumble, 0);
	}
}
