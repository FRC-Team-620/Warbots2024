package org.jmhsrobotics.frc2024.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;

public class ClimberSubsystem extends SubsystemBase implements Logged {
	private CANSparkMax climber, helper;
	private RelativeEncoder climberEncoder;

	public ClimberSubsystem() {
		climber = new CANSparkMax(60, MotorType.kBrushless);
		helper = new CANSparkMax(61, MotorType.kBrushless);
		climberEncoder = climber.getEncoder();

		climber.restoreFactoryDefaults();
		helper.restoreFactoryDefaults();
		climber.setSmartCurrentLimit(40);
		helper.setSmartCurrentLimit(40);
		climber.setIdleMode(IdleMode.kBrake);
		helper.setIdleMode(IdleMode.kBrake);
		climberEncoder.setPositionConversionFactor((5.0 * 4.0) / 100.0); // 20:1 gear reduction

		// climber.setSoftLimit(SoftLimitDirection.kReverse, 10);
		// climber.setSoftLimit(SoftLimitDirection.kForward, 40);
		// climber.enableSoftLimit(SoftLimitDirection.kForward, true);
		// climber.enableSoftLimit(SoftLimitDirection.kReverse, true);

		// climber.setInverted(true);
		helper.follow(climber, true);
	}

	public void set(double amount) {
		this.climber.set(amount);
	}

	public double getEncoderPostition() {
		return this.climberEncoder.getPosition();
	}
	@Override
	public void periodic() {
		// SmartDashboard.putNumber("ClimberSubsystem/EncoderReading",
		// getEncoderPostition());
		// TODO Auto-generated method stub
		log("climberAppliedOutput", climber.getAppliedOutput());
		log("climberHelperAppliedOutput", climber.getAppliedOutput());
		// log("climberPosition", encoder.getPosition());
		super.periodic();
	}

	// TEMP CODE
	public void extend() {
		climber.set(0.3);
	}

	public void retract() {
		climber.set(-0.3);
	}

	public void stop() {
		climber.set(0);
	}
}
