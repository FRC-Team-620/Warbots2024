package org.jmhsrobotics.frc2024.subsystems.climber;

import org.jmhsrobotics.frc2024.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;

public class ClimberSubsystem extends SubsystemBase implements Logged {
	private CANSparkMax leftClimber, rightClimber;
	private RelativeEncoder leftClimberEncoder, rightClimberEncoder;

	public ClimberSubsystem() {
		leftClimber = new CANSparkMax(Constants.CAN.kLeftClimberID, MotorType.kBrushless);
		rightClimber = new CANSparkMax(Constants.CAN.kRightClimberID, MotorType.kBrushless);
		leftClimberEncoder = leftClimber.getEncoder();
		rightClimberEncoder = rightClimber.getEncoder();

		leftClimber.restoreFactoryDefaults();
		rightClimber.restoreFactoryDefaults();
		leftClimber.setSmartCurrentLimit(40);
		rightClimber.setSmartCurrentLimit(40);
		leftClimber.setSoftLimit(SoftLimitDirection.kReverse, 0);
		rightClimber.setSoftLimit(SoftLimitDirection.kReverse, 0);
		leftClimber.setSoftLimit(SoftLimitDirection.kForward, 100);
		rightClimber.setSoftLimit(SoftLimitDirection.kForward, 0);
		setSoftLimit(true);
		leftClimber.setIdleMode(IdleMode.kBrake);
		rightClimber.setIdleMode(IdleMode.kBrake);
		leftClimberEncoder.setPositionConversionFactor((5.0 * 4.0) / 100.0); // 20:1 gear reduction
		rightClimberEncoder.setPositionConversionFactor((5.0 * 4.0) / 100.0); // 20:1 gear reduction

		// climber.setInverted(true);

		// rightClimber.follow(leftClimber, true);
	}

	public void setSoftLimit(boolean toggle) {
		rightClimber.enableSoftLimit(SoftLimitDirection.kForward, toggle);
		leftClimber.enableSoftLimit(SoftLimitDirection.kForward, toggle);
		rightClimber.enableSoftLimit(SoftLimitDirection.kReverse, toggle);
		leftClimber.enableSoftLimit(SoftLimitDirection.kReverse, toggle);
	}

	public void setLeftMotor(double amount) {
		this.leftClimber.set(amount);
	}

	public void setRightMotor(double amount) {
		this.rightClimber.set(amount);
	}

	public double getLeftEncoderPostition() {
		return this.leftClimberEncoder.getPosition();
	}

	public void climberRetract() {
		this.leftClimber.setVoltage(-10);
		this.rightClimber.setVoltage(-10);
	}

	public void climberExtend() {
		this.leftClimber.setVoltage(10);
		this.rightClimber.setVoltage(10);
	}

	public void climberStop() {
		this.leftClimber.setVoltage(0);
		this.rightClimber.setVoltage(0);
	}
	public double getRightEncoderPosition() {
		return this.rightClimberEncoder.getPosition();
	}
	@Override
	public void periodic() {
		super.periodic();
	}

}
