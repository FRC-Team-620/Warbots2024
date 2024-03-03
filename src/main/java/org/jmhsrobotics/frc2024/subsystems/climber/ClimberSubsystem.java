package org.jmhsrobotics.frc2024.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;

public class ClimberSubsystem extends SubsystemBase implements Logged {
<<<<<<< Updated upstream
	private CANSparkMax climber, helper;
	private RelativeEncoder encoder;

	public ClimberSubsystem() {
		climber = new CANSparkMax(60, MotorType.kBrushless);
		helper = new CANSparkMax(61, MotorType.kBrushless);
		encoder = climber.getEncoder();

		climber.restoreFactoryDefaults();
		helper.restoreFactoryDefaults();
		climber.setSmartCurrentLimit(40);
		helper.setSmartCurrentLimit(40);
		climber.setIdleMode(IdleMode.kBrake);
		helper.setIdleMode(IdleMode.kBrake);
		encoder.setPositionConversionFactor((5.0 * 4.0) / 100.0); // 20:1 gear reduction
=======
	private CANSparkMax leftClimber, rightClimber;
	private RelativeEncoder leftClimberEncoder, rightClimberEncoder;

	public ClimberSubsystem() {
		leftClimber = new CANSparkMax(60, MotorType.kBrushless);
		rightClimber = new CANSparkMax(61, MotorType.kBrushless);
		leftClimberEncoder = leftClimber.getEncoder();
		rightClimberEncoder = rightClimber.getEncoder();

		leftClimber.restoreFactoryDefaults();
		rightClimber.restoreFactoryDefaults();
		leftClimber.setSmartCurrentLimit(40);
		rightClimber.setSmartCurrentLimit(40);
		leftClimber.setIdleMode(IdleMode.kBrake);
		rightClimber.setIdleMode(IdleMode.kBrake);
		leftClimberEncoder.setPositionConversionFactor((5.0 * 4.0) / 100.0); // 20:1 gear reduction
>>>>>>> Stashed changes

		// climber.setSoftLimit(SoftLimitDirection.kReverse, 10);
		// climber.setSoftLimit(SoftLimitDirection.kForward, 40);
		// climber.enableSoftLimit(SoftLimitDirection.kForward, true);
		// climber.enableSoftLimit(SoftLimitDirection.kReverse, true);

		// climber.setInverted(true);

		// rightClimber.follow(leftClimber, true);
	}

<<<<<<< Updated upstream
	@Override
	public void periodic() {
		// TODO Auto-generated method stub
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
=======
	public void setLeftMotor(double amount) {
		this.leftClimber.set(amount);
	}

	public void setRightMotor(double amount) {
		this.rightClimber.set(amount);
	}

	public double getLeftEncoderPostition() {
		return this.leftClimberEncoder.getPosition();
	}

	public double getRightEncoderPosition() {
		return this.rightClimberEncoder.getPosition();
	}
	@Override
	public void periodic() {
		SmartDashboard.putNumber("ClimberSubsystem/LeftEncoderReading", getLeftEncoderPostition());
		SmartDashboard.putNumber("ClimberSubsystem/RightClimberEncoderPosition", getLeftEncoderPostition());
>>>>>>> Stashed changes
	}
}
