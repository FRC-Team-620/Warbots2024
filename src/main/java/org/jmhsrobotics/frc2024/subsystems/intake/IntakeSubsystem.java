package org.jmhsrobotics.frc2024.subsystems.intake;

import org.jmhsrobotics.frc2024.Constants;
import org.jmhsrobotics.warcore.rev.RevEncoderSimWrapper;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
	private CANSparkMax intakeMotor;
	private DigitalInput intakeSwitch;

	public IntakeSubsystem() {
		intakeMotor = new CANSparkMax(Constants.CAN.kIntakeId, MotorType.kBrushless);
		intakeMotor.setInverted(true);
		intakeMotor.setIdleMode(IdleMode.kBrake);

		intakeMotor.setSmartCurrentLimit(35);

		intakeSwitch = new DigitalInput(Constants.DIO.kIntakeSwitch);
		if (RobotBase.isSimulation()) {
			simInit();
		}
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("intake/velocityRPM", intakeMotor.getEncoder().getVelocity());
		SmartDashboard.putNumber("intake/currentDrawAmps", intakeMotor.getOutputCurrent());
		SmartDashboard.putBoolean("intake/hasNote", this.hasNote());
	}

	public void set(double speed) {
		intakeMotor.set(speed);
	}

	public boolean hasNote() {
		return intakeSwitch.get();
	}

	private DIOSim intakeSwitchSim;
	private DCMotorSim intakeSim;
	private RevEncoderSimWrapper intakeEncSim;

	public void simInit() {
		intakeSwitchSim = new DIOSim(Constants.DIO.kIntakeSwitch);
		intakeSim = new DCMotorSim(DCMotor.getNEO(1), 1, 0.3);
		intakeEncSim = RevEncoderSimWrapper.create(intakeMotor);
	}

	@Override
	public void simulationPeriodic() {
		double intakeVolts = intakeMotor.get() * 12;
		intakeSim.setInput(intakeVolts);
		intakeSim.update(Constants.ksimDtSec);
		intakeSwitchSim.setValue(true); // TODO placeholder.
		intakeEncSim.setDistance(intakeSim.getAngularPositionRotations());
		intakeEncSim.setVelocity(intakeSim.getAngularVelocityRPM());

	}
}
