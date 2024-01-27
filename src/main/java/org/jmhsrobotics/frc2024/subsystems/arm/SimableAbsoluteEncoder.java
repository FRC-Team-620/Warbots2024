package org.jmhsrobotics.frc2024.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.RobotBase;

public class SimableAbsoluteEncoder implements AbsoluteEncoder {

	private AbsoluteEncoder enc;
	private double position;

	SimableAbsoluteEncoder(AbsoluteEncoder enc) {
		this.enc = enc;
	}

	@Override
	public double getPosition() {
		if (RobotBase.isSimulation()) {
			return position;
		} else {
			return enc.getPosition();
		}

	}

	@Override
	public double getVelocity() {
		return enc.getVelocity();
	}

	@Override
	public REVLibError setPositionConversionFactor(double factor) {
		return enc.setPositionConversionFactor(factor);
	}

	@Override
	public double getPositionConversionFactor() {
		return enc.getPositionConversionFactor();
	}

	@Override
	public REVLibError setVelocityConversionFactor(double factor) {
		return enc.setVelocityConversionFactor(factor);
	}

	@Override
	public double getVelocityConversionFactor() {
		return enc.getVelocityConversionFactor();
	}

	@Override
	public REVLibError setInverted(boolean inverted) {
		return enc.setInverted(inverted);
	}

	@Override
	public boolean getInverted() {
		return enc.getInverted();
	}

	@Override
	public REVLibError setAverageDepth(int depth) {
		return enc.setAverageDepth(depth);
	}

	@Override
	public int getAverageDepth() {
		return enc.getAverageDepth();
	}

	@Override
	public REVLibError setZeroOffset(double offset) {
		return enc.setZeroOffset(offset);
	}

	@Override
	public double getZeroOffset() {
		return enc.getZeroOffset();
	}

	public void setPosition(double position) {
		this.position = position;
	}

}
