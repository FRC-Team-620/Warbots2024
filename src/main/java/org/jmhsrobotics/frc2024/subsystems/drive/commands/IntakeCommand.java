package org.jmhsrobotics.frc2024.subsystems.drive.commands;

import com.revrobotics.CANSparkMax;

import org.jmhsrobotics.frc2024.subsystems.drive.DriveSubsystem;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {
	protected CANSparkMax innerIntakeMotor;
	protected Solenoid intakeArmsVirtualSolenoidA;
	protected Solenoid intakeArmsVirtualSolenoidB;
	protected CANSparkMax intakeArmsMotor;
	protected DigitalInput intakeSwitch;

	// protected boolean isClimbing;
	public IntakeCommand(DriveSubsystem driveSubsystem, int i) {
		innerIntakeMotor = new CANSparkMax(IntakeConstants.innerIntakeMotor, MotorType.kBrushless);
		innerIntakeMotor.restoreFactoryDefaults();
		innerIntakeMotor.setInverted(true);
		innerIntakeMotor.setIdleMode(IdleMode.kBrake);

		innerIntakeMotor.setSmartCurrentLimit(35);

		intakeArmsVirtualSolenoidA = new Solenoid(PneumaticsModuleType.CTREPCM, 3);
		intakeArmsVirtualSolenoidB = new Solenoid(PneumaticsModuleType.CTREPCM, 4);
		intakeArmsVirtualSolenoidA.set(false);
		intakeArmsVirtualSolenoidB.set(false);

		intakeArmsMotor = new CANSparkMax(IntakeConstants.intakeArmsMotor, MotorType.kBrushless);
		intakeArmsMotor.restoreFactoryDefaults();
		intakeArmsMotor.setInverted(true);

		intakeArmsMotor.setSmartCurrentLimit(35);

		intakeSwitch = new DigitalInput(IntakeConstants.intakeSwitch);
	}
	public void enableInnerIntakeMotor() {
		innerIntakeMotor.set(1);
	}
	public void reverseInnerIntakeMotor() {
		innerIntakeMotor.set(-1);
	}
	public void disableInnerIntakeMotor() {
		innerIntakeMotor.set(0);
	}
	public void setInnerIntakeMotor(double speed) {
		this.innerIntakeMotor.set(speed);
	}

	// implementing 836's idea
	public void extendIntakeArms() {
		intakeArmsVirtualSolenoidA.set(true);
		intakeArmsVirtualSolenoidB.set(true);
	}

	// implementing 836's idea
	public void retractIntakeArms() {
		intakeArmsVirtualSolenoidA.set(false);
		intakeArmsVirtualSolenoidB.set(false);
	}

	// implementing 836's idea
	public void floatIntakeArms() {
		intakeArmsVirtualSolenoidA.set(false);
		intakeArmsVirtualSolenoidB.set(true);
	}

	public void enableIntakeArmsMotor() {
		intakeArmsMotor.set(-0.6);
	}
	public void disableIntakeArmsMotor() {
		intakeArmsMotor.set(0);
	}
	public void reverseIntakeArmsMotor() {
		intakeArmsMotor.set(0.6);
	}
	public double extendedIntakeArmsSpeed() {
		return intakeArmsMotor.get();
	}

	public boolean getIntakeSwitch() {
		return intakeSwitch.get();
	}
}
