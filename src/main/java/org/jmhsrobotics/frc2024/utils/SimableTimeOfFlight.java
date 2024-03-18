package org.jmhsrobotics.frc2024.utils;

import org.jmhsrobotics.frc2024.Robot;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimEnum;
import edu.wpi.first.hal.SimDevice.Direction;

public class SimableTimeOfFlight extends TimeOfFlight {
	private SimDevice tof;
	private SimBoolean rangeValid;
	private SimDouble range, rangeSigma, ambientLight, sampleTime;
	private SimEnum status, rangeMode;
	private static final String[] STATUS_STRINGS = new String[]{"Valid", "SigmaHigh", "ReturnSignalLow",
			"ReturnPhaseBad", "HardwareFailure", "WrappedTarget", "InternalError", "Invalid"};
	private static final String[] MODE_STRINGS = new String[]{"Short", "Medium", "Long"};

	public final int canId;
	public static class Sim {
		public static final String kDeviceName = "Time Of Flight";
		public static final String kRangeValid = "Range Valid";
		public static final String kRange = "Range";
		public static final String kRangeSigma = "Range Sigma";
		public static final String kAmbientLight = "Ambient Light Level";
		public static final String kStatus = "Status";
		public static final String kRangeMode = "Range Mode";
		public static final String kSampleTime = "Sample Time";
		public static final String kROITopLeft = "Range Of Interest Top Left";
		public static final String kROITopRight = "Range Of Interest Top Right";
		public static final String kROIBottomLeft = "Range Of Interest Lower Left";
		public static final String kROIBottomRight = "Range Of Interest Lower Right";
	}

	public SimableTimeOfFlight(int sensorId) {

		super(sensorId);
		this.canId = sensorId;
		if (Robot.isSimulation()) {
			tof = SimDevice.create(Sim.kDeviceName, sensorId);
			rangeValid = tof.createBoolean(Sim.kRangeValid, Direction.kBidir, true);
			range = tof.createDouble(Sim.kRange, Direction.kBidir, 0);
			rangeSigma = tof.createDouble(Sim.kRangeSigma, Direction.kBidir, 0);
			ambientLight = tof.createDouble(Sim.kAmbientLight, Direction.kBidir, 0);

			status = tof.createEnum(Sim.kStatus, Direction.kBidir, SimableTimeOfFlight.STATUS_STRINGS, 0);
			rangeMode = tof.createEnum(Sim.kRangeMode, Direction.kBidir, SimableTimeOfFlight.MODE_STRINGS, 1);
			sampleTime = tof.createDouble(Sim.kSampleTime, Direction.kBidir, 24);
			tof.createInt(Sim.kROITopLeft, Direction.kBidir, 0);
			tof.createInt(Sim.kROITopRight, Direction.kBidir, 0);
			tof.createInt(Sim.kROIBottomLeft, Direction.kBidir, 0);
			tof.createInt(Sim.kROIBottomRight, Direction.kBidir, 0);
		}

	}

	@Override
	public boolean isRangeValid() {
		if (Robot.isSimulation()) {
			return this.rangeValid.get();
		}
		return super.isRangeValid();
	}

	@Override
	public double getRange() {
		if (Robot.isSimulation()) {
			return this.range.get();
		}
		return super.getRange();
	}

	@Override
	public double getRangeSigma() {
		if (Robot.isSimulation()) {
			return this.rangeSigma.get();
		}
		return super.getRangeSigma();
	}

	@Override
	public double getAmbientLightLevel() {
		if (Robot.isSimulation()) {
			return this.ambientLight.get();
		}
		return super.getAmbientLightLevel();
	}

	@Override
	public Status getStatus() {
		if (Robot.isSimulation()) {
			int statusIndex = this.status.get();
			Status status;
			if (statusIndex == 0) {
				status = Status.Valid;
			} else if (statusIndex == 1) {
				status = Status.SigmaHigh;
			} else if (statusIndex == 2) {
				status = Status.ReturnSignalLow;
			} else if (statusIndex == 4) {
				status = Status.ReturnPhaseBad;
			} else if (statusIndex == 5) {
				status = Status.HardwareFailure;
			} else if (statusIndex == 7) {
				status = Status.WrappedTarget;
			} else if (statusIndex == 8) {
				status = Status.InternalError;
			} else {
				status = Status.Invalid;
			}
			return status;

		}
		return super.getStatus();
	}

	@Override
	public RangingMode getRangingMode() {
		if (Robot.isSimulation()) {
			int modeIndex = this.rangeMode.get();
			RangingMode mode;
			if (modeIndex == 2) {
				mode = RangingMode.Long;
			} else if (modeIndex == 1) {
				mode = RangingMode.Medium;
			} else {
				mode = RangingMode.Short;
			}
			return mode;

		}
		return super.getRangingMode();
	}

	@Override
	public double getSampleTime() {
		if (Robot.isSimulation()) {
			return this.sampleTime.get();
		}
		return super.getSampleTime();
	}

	@Override
	public void setRangingMode(RangingMode mode, double sampleTime) {
		if (Robot.isSimulation()) {
			int apiMode;

			if (mode == RangingMode.Long) {
				apiMode = 2;
			} else if (mode == RangingMode.Medium) {
				apiMode = 1;
			} else {
				apiMode = 0;
			}
			this.rangeMode.set(apiMode);
			this.sampleTime.set(sampleTime);
		}
		super.setRangingMode(mode, sampleTime);
	}

}
