package org.jmhsrobotics.frc2024.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkBase.IdleMode;

public class ArmSubsystem extends SubsystemBase {

	private MechanismLigament2d m_arm;
	private CANSparkMax armPivot = new CANSparkMax(9, MotorType.kBrushless);
	private AbsoluteEncoder pitchEncoder;
	private Mechanism2d mech;
	

	public ArmSubsystem() {
		pitchEncoder = armPivot.getAbsoluteEncoder(Type.kDutyCycle);
		armPivot.setSmartCurrentLimit(40);

		armPivot.setIdleMode(IdleMode.kBrake);
		init2d();
	}

	public void setArmPivot(double amount) {
		armPivot.set(amount);
		// SmartDashboard.putNumber("ArmSubsystem/data/ArmPivotSpeed", amount);
	}

	public double getArmPitch() {
		return this.pitchEncoder.getPosition();
	}

	private SparkAnalogSensor pitchencsim;

	public void init2d() {
		// TODO: finish sim
		mech = new Mechanism2d(3, 3);
		MechanismRoot2d root = mech.getRoot("base", 1.5, 1.5);
		m_arm = root.append(new MechanismLigament2d("arm", 2, 0));
		// pitchencsim = new SparkAnalogSensor(pitchEncoder);

	}

	@Override
	public void periodic() {
		// TODO Auto-generated method stub
		super.periodic();
		m_arm.setAngle(getArmPitch());
		SmartDashboard.putData("ArmSubsystem/armSIM", mech);
		SmartDashboard.putNumber("ArmSubsystem/velocity", this.pitchEncoder.getVelocity());
		SmartDashboard.putString("ArmSubsystem/encoder", pitchEncoder.toString());

	} 

}
