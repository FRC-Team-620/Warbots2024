package org.jmhsrobotics.frc2024.subsystems.intake;

import org.jmhsrobotics.frc2024.subsystems.intake.SimableTimeOfFlight.Sim;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimEnum;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class SimTimeOfFlight {

	SimableTimeOfFlight tof;
	SimDeviceSim sim;
	private SimBoolean rangeValid;
	private SimDouble range, rangeSigma, ambientLight, sampleTime;
	private SimEnum status, rangeMode;

	public SimTimeOfFlight(SimableTimeOfFlight tof) {
		sim = new SimDeviceSim(Sim.kDeviceName, tof.canId);
		rangeValid = sim.getBoolean(Sim.kRangeValid);
		range = sim.getDouble(Sim.kRange);
		rangeSigma = sim.getDouble(Sim.kRangeSigma);
		ambientLight = sim.getDouble(Sim.kAmbientLight);
		sampleTime = sim.getDouble(Sim.kSampleTime);
		status = sim.getEnum(Sim.kStatus);
		rangeMode = sim.getEnum(Sim.kRangeMode);
	}

	public void setRange(double rangeVal) {
		this.range.set(rangeVal);
	}

	public void setRangeValid(boolean isValid) {
		this.rangeValid.set(isValid);
	}
	public void setRangeSigma(double sigma) {
		this.rangeSigma.set(sigma);
	}
	public void setAmientLightLevel(double level) {
		this.ambientLight.set(level);
	}
	public void setSampleTime(double period) {
		this.sampleTime.set(period);
	}
	// TODO: Set Status and Set rangemode
	// TODO: set ROI
}
