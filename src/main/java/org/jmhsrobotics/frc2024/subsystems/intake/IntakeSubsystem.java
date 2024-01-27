package org.jmhsrobotics.frc2024.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
	private CANSparkMax intakeMotor;
	private DigitalInput intakeSwitch;

	public IntakeSubsystem() {
		intakeMotor = new CANSparkMax(32, MotorType.kBrushless); // TODO: move to constants file
		intakeMotor.setInverted(true);
		intakeMotor.setIdleMode(IdleMode.kBrake);

		intakeMotor.setSmartCurrentLimit(35);

		intakeSwitch = new DigitalInput(2); // TODO: move to constants file
	}

	public void set(double speed) {
		intakeMotor.set(speed);
	}

	public boolean hasNote() {
		return intakeSwitch.get();
	}
}
