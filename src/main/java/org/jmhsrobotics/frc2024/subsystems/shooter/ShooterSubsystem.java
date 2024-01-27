package org.jmhsrobotics.frc2024.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
	private CANSparkMax motor1 = new CANSparkMax(7, MotorType.kBrushless);
	private CANSparkMax motor2 = new CANSparkMax(8, MotorType.kBrushless);;
	private RelativeEncoder encoder;

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

	public double getRPM() {
		encoder = motor1.getEncoder();
		return encoder.getVelocity();
	}

	public void setSpeed(double speed) {
		motor1.set(speed);
	}
}
