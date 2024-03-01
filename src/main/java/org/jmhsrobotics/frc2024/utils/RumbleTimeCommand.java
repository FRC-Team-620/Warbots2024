package org.jmhsrobotics.frc2024.utils;

import org.jmhsrobotics.frc2024.controlBoard.ControlBoard;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class RumbleTimeCommand extends Command {

	/**
	 * A weak rumble on both sides of the controller
	 *
	 * @param controller
	 *            - the xbox controller that we're rumbling
	 */

	private ControlBoard board;
	private RumbleType type;
	private double time, strength;

	private Timer timer = new Timer();
	public RumbleTimeCommand(ControlBoard board, RumbleType type, double time, double strength) {
		this.board = board;
		this.type = type;
		this.time = time;
		this.strength = strength;
	}

	@Override
	public void initialize() {
		board.setRumble(this.type, this.strength);
		this.timer.reset();
		timer.start();
	}

	@Override
	public void execute() {
		board.setRumble(this.type, this.strength);
	}

	@Override
	public boolean isFinished() {
		return this.timer.hasElapsed(time);
	}

	@Override
	public void end(boolean interrupted) {
		board.setRumble(type, 0);
	}
}
