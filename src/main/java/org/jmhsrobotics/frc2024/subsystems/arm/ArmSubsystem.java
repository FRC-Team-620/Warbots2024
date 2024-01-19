package org.jmhsrobotics.frc2024.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

	private MechanismLigament2d m_arm;
	private CANSparkMax armPivot = new CANSparkMax(9, MotorType.kBrushless);
	private RelativeEncoder PitchEncoder = armPivot.getEncoder();
	// private AbsoluteEncoder PitchEncoder=armPivot.getAbsoluteEncoder(kDutyCycle);
	// private AbsoluteEncoder PitchEncoder=armPivot.getEncoder();

	public ArmSubsystem() {

		armPivot.setSmartCurrentLimit(40);
		armPivot.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);

	}

	public void setArmPivot(double amount) {
		armPivot.set(amount);
	}

	public double getArmPitch() {
		return PitchEncoder.getPosition();
	}

	public void init2d() {
		Mechanism2d mech = new Mechanism2d(3, 3);
		MechanismRoot2d root = mech.getRoot("base", 1.5, 0);
		var m_arm = root.append(new MechanismLigament2d("arm", 2, 0));
		SmartDashboard.putData("Mech2d", mech);
	}

	@Override
	public void periodic() {
		// TODO Auto-generated method stub
		super.periodic();
		m_arm.setAngle(getArmPitch());
	}

}
