package org.jmhsrobotics.frc2024.subsystems.climber;

import org.jmhsrobotics.frc2024.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
		// TODO: input real vals for soft limit
		leftClimber.setSoftLimit(SoftLimitDirection.kReverse, 0);
		rightClimber.setSoftLimit(SoftLimitDirection.kReverse, 0);
		leftClimber.setSoftLimit(SoftLimitDirection.kForward, 34);
		rightClimber.setSoftLimit(SoftLimitDirection.kForward, 34);
		setSoftLimit(true);
		leftClimber.setIdleMode(IdleMode.kBrake);
		rightClimber.setIdleMode(IdleMode.kBrake);
		leftClimberEncoder.setPositionConversionFactor((5.0 * 4.0) / 100.0); // 20:1 gear reduction
		rightClimberEncoder.setPositionConversionFactor((5.0 * 4.0) / 100.0); // 20:1 gear reduction

		SmartDashboard.putNumber("climber/leftEncoder", 0);
		SmartDashboard.putNumber("climber/rightEncoder", 0);

		// climber.setInverted(true);

		// rightClimber.follow(leftClimber, true);
	}

	public void setSoftLimit(boolean toggle) {
		rightClimber.enableSoftLimit(SoftLimitDirection.kForward, toggle);
		leftClimber.enableSoftLimit(SoftLimitDirection.kForward, toggle);
		rightClimber.enableSoftLimit(SoftLimitDirection.kReverse, toggle);
		leftClimber.enableSoftLimit(SoftLimitDirection.kReverse, toggle);
		// rightClimberEncoder.setPosition(0);
		// leftClimberEncoder.setPosition(0);

	}

	public void setLeftMotor(double amount) {
		this.leftClimber.set(amount);
	}

	public void setRightMotor(double amount) {
		this.rightClimber.set(amount);
	}

	public void climberRetract() {
		this.leftClimber.setVoltage(-10);
		this.rightClimber.setVoltage(-10);
		SmartDashboard.putNumber("climber/leftEncoder", getLeftEncoderPostition());
		SmartDashboard.putNumber("climber/rightEncoder", getRightEncoderPosition());
	}

	public void climberExtend() {
		this.leftClimber.setVoltage(10);
		this.rightClimber.setVoltage(10);
		SmartDashboard.putNumber("climber/leftEncoder", getLeftEncoderPostition());
		SmartDashboard.putNumber("climber/rightEncoder", getRightEncoderPosition());
	}

	public void climberStop() {
		this.leftClimber.setVoltage(0);
		this.rightClimber.setVoltage(0);
	}
	public double getRightEncoderPosition() {
		return this.rightClimberEncoder.getPosition();
	}

	public double getLeftEncoderPostition() {
		return this.leftClimberEncoder.getPosition();
	}

	@Override
	public void periodic() {
		super.periodic();
	}

}
