package org.jmhsrobotics.frc2024.subsystems.arm;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.RobotBase;

public class SimableRelativeEncoder implements RelativeEncoder {

	private RelativeEncoder enc;
	private double position;

	SimableRelativeEncoder(RelativeEncoder enc) {
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
	public REVLibError setPosition(double position) {
		this.position = position;
		return enc.setPosition(position);
	}

	@Override
	public REVLibError setMeasurementPeriod(int period_ms) {
		return enc.setMeasurementPeriod(period_ms);
	}

	@Override
	public int getMeasurementPeriod() {
		return enc.getMeasurementPeriod();
	}

	@Override
	public int getCountsPerRevolution() {
		return enc.getCountsPerRevolution();
	}
}
