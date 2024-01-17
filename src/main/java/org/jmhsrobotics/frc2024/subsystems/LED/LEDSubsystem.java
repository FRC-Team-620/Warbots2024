package org.jmhsrobotics.frc2024.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
	private AddressableLED LED;
	private AddressableLEDBuffer ledBuffer;

	public LEDSubsystem() {
		if (RobotBase.isSimulation()) {
			initSim();
		}

		// TODO: find the real port
		this.LED = new AddressableLED(9);

		// TODO: get the real length
		this.ledBuffer = new AddressableLEDBuffer(60);

		this.LED.setLength(this.ledBuffer.getLength());
		this.LED.setData(this.ledBuffer);

	}

	public void startLED() {
		this.LED.start();
	}

	public void endLED() {
		this.LED.stop();;
	}

	public void setRGB(int r, int g, int b) {
		for (int i = 0; i < this.ledBuffer.getLength(); i++) {
			this.ledBuffer.setRGB(i, r, g, b);
		}
	}

	public void rainBow() {
		int rainBowPixelHue = 0;
		for (var i = 0; i < this.ledBuffer.getLength(); i++) {
			final var hue = (rainBowPixelHue + (i * 180 / this.ledBuffer.getLength())) % 180;
			this.ledBuffer.setHSV(i, hue, 225, 128);
		}
		rainBowPixelHue += 3;

		rainBowPixelHue %= 180;
	}

	private AddressableLEDSim simLEDs;
	private void initSim() {
		simLEDs = new AddressableLEDSim(LED);
		simLEDs.setLength(60);

	}

	@Override
	public void simulationPeriodic() {
	}
}
