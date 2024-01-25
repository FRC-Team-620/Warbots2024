package org.jmhsrobotics.frc2024.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
	private CANSparkMax motor1;
	private CANSparkMax motor2;

	// private double speed;

	public ShooterSubsystem() {
		// Initializes motor(s)
		initializeMotors();
	}

	private void initializeMotors() {
		motor1.setIdleMode(IdleMode.kCoast);
		motor1.setSmartCurrentLimit(20);
		motor1.setOpenLoopRampRate(20);
		motor2.setIdleMode(IdleMode.kCoast);
		motor2.setSmartCurrentLimit(20);
		motor2.setOpenLoopRampRate(20);

		motor2.follow(motor1);
	}

	public void setSpeed(double speed) {
		motor1.set(speed);
	}
}
