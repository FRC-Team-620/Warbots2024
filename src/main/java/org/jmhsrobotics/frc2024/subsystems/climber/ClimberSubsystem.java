package org.jmhsrobotics.frc2024.subsystems.climber;

import org.jmhsrobotics.frc2024.Constants;
import org.jmhsrobotics.frc2024.Constants.CAN;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

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
		leftClimber.setIdleMode(IdleMode.kBrake);
		rightClimber.setIdleMode(IdleMode.kBrake);
		leftClimberEncoder.setPositionConversionFactor((5.0 * 4.0) / 100.0); // 20:1 gear reduction
		rightClimberEncoder.setPositionConversionFactor((5.0 * 4.0) / 100.0); // 20:1 gear reduction

		// optimize Can Traffic
		// leftClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10); // applied
		// output, faults, sticky faults, isfollower - 10ms
		leftClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200); // Velocity, temp, voltage,current - 20ms
		leftClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 200); // Rel pos - 20ms
		leftClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus3, CAN.kMaxFramePeriodMs); // Analog sensor volts, vel,
																							// acc - 50ms
		leftClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus4, CAN.kMaxFramePeriodMs); // Alternate Encoder Vel/pos
																							// - 20ms
		leftClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus5, CAN.kMaxFramePeriodMs); // Duty Cycle Absolute
																							// Encoder Position/
		// angle - 200ms
		leftClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus6, CAN.kMaxFramePeriodMs); // Duty Cycle Absolute
																							// Encoder Velocity/
		// freequency - 200ms
		leftClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus7, CAN.kMaxFramePeriodMs); // I accumm

		// optimize Can Traffic
		// rightClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10); // applied
		// output, faults, sticky faults, isfollower - 10ms
		rightClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200); // Velocity, temp, voltage,current - 20ms
		rightClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 200); // Rel pos - 20ms
		rightClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus3, CAN.kMaxFramePeriodMs); // Analog sensor volts, vel,
																							// acc - 50ms
		rightClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus4, CAN.kMaxFramePeriodMs); // Alternate Encoder Vel/pos
																							// - 20ms
		rightClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus5, CAN.kMaxFramePeriodMs); // Duty Cycle Absolute
																							// Encoder Position/
		// angle - 200ms
		rightClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus6, CAN.kMaxFramePeriodMs); // Duty Cycle Absolute
																							// Encoder Velocity/
		// freequency - 200ms
		rightClimber.setPeriodicFramePeriod(PeriodicFrame.kStatus7, CAN.kMaxFramePeriodMs); // I accumm

		// climber.setSoftLimit(SoftLimitDirection.kReverse, 10);
		// climber.setSoftLimit(SoftLimitDirection.kForward, 40);
		// climber.enableSoftLimit(SoftLimitDirection.kForward, true);
		// climber.enableSoftLimit(SoftLimitDirection.kReverse, true);

		// climber.setInverted(true);

		// rightClimber.follow(leftClimber, true);
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

	public double getRightEncoderPosition() {
		return this.rightClimberEncoder.getPosition();
	}
	@Override
	public void periodic() {
		super.periodic();
	}

}
