package org.jmhsrobotics.frc2024.subsystems.arm;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkBase.IdleMode;

public class ArmSubsystem extends SubsystemBase {

	private MechanismLigament2d m_arm;
	private CANSparkMax armPivot = new CANSparkMax(9, MotorType.kBrushless);
	private SimableAbsoluteEncoder pitchEncoder;
	private Mechanism2d mech;

	public ArmSubsystem() {
		pitchEncoder = new SimableAbsoluteEncoder(armPivot.getAbsoluteEncoder(Type.kDutyCycle));
		armPivot.setSmartCurrentLimit(40);
		armPivot.setIdleMode(IdleMode.kBrake);
		init2d();
		if (RobotBase.isSimulation()) {
			initSim();
		}
	}

	public void setArmPivot(double amount) {
		armPivot.set(amount);
		// SmartDashboard.putNumber("ArmSubsystem/data/ArmPivotSpeed", amount);
	}

	public double getArmPitch() {
		return this.pitchEncoder.getPosition();
	}

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
		SmartDashboard.putNumber("ArmSubsystem/encoder", pitchEncoder.getPosition());

	}
	SingleJointedArmSim armSim;
	public void initSim() {
		double armGearRatio = 1;
		double moi = 1;
		armSim = new SingleJointedArmSim(DCMotor.getNEO(2), armGearRatio, moi, Units.inchesToMeters(23), 0,
				Units.degreesToRadians(180), false, 0);
		// simEncoder = new RevEncoderSimWrapper(null, null);
	}
	@Override
	public void simulationPeriodic() {
		double armVolts = armPivot.get() * 12;
		armSim.setInputVoltage(armVolts);
		pitchEncoder.setPosition(Units.radiansToDegrees(armSim.getAngleRads()));
	}

}
