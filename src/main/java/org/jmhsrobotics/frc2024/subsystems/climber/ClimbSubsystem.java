package org.jmhsrobotics.frc2024.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase.IdleMode;

public class ClimbSubsystem extends SubsystemBase {

	private CANSparkMax climbMotor = new CANSparkMax(11, MotorType.kBrushless);
	private RelativeEncoder climbEncoder = climbMotor.getEncoder();

	public ClimbSubsystem() {
		climbEncoder.setPosition(0);
		climbMotor.setIdleMode(IdleMode.kBrake);
		climbMotor.setSmartCurrentLimit(40);

	}

	public CANSparkMax getMotor() {
		return climbMotor;
	}

	public void setMotor(double amount) {
		climbMotor.set(amount);
	}

	public double getClimbPosition() {
		return climbEncoder.getPosition();
	}

	public void setEncoder(double amount) {
		climbEncoder.setPosition(amount);
	}

	public RelativeEncoder getEncoder() {
		return climbEncoder;
	}

}
