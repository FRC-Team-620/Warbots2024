package org.jmhsrobotics.frc2024.subsystems.intake;

import org.jmhsrobotics.frc2024.Constants;
import org.jmhsrobotics.frc2024.Robot;
import org.jmhsrobotics.frc2024.utils.SimTimeOfFlight;
import org.jmhsrobotics.frc2024.utils.SimableTimeOfFlight;
import org.jmhsrobotics.warcore.rev.RevEncoderSimWrapper;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;

public class IntakeSubsystem extends SubsystemBase implements Logged {
	private CANSparkMax intakeMotor;

	private SimableTimeOfFlight lowerSensor;
	private SimableTimeOfFlight upperSensor;

	private boolean isIntaking = false;

	public IntakeSubsystem() {
		intakeMotor = new CANSparkMax(Constants.CAN.kIntakeId, MotorType.kBrushless);
		intakeMotor.setInverted(true);
		intakeMotor.setIdleMode(IdleMode.kBrake);

		this.lowerSensor = new SimableTimeOfFlight(1);
		this.upperSensor = new SimableTimeOfFlight(0);
		intakeMotor.setSmartCurrentLimit(35);

		this.lowerSensor.setRangingMode(RangingMode.Short, 24);
		this.upperSensor.setRangingMode(RangingMode.Short, 24);
		if (RobotBase.isSimulation()) {
			simInit();
		}
	}

	public boolean isIntaking() {
		return isIntaking;
	}

	@Override
	public void periodic() {
		this.isIntaking = this.intakeMotor.get() != 0 ? true : false;
		// SmartDashboard.putNumber("intake/velocityRPM",
		// intakeMotor.getEncoder().getVelocity());
		// SmartDashboard.putNumber("intake/currentDrawAmps",
		// intakeMotor.getOutputCurrent());
		// SmartDashboard.putBoolean("Intake/highSwitchState",
		// this.highSwitch().isPressed());
		// SmartDashboard.putBoolean("Intake/lowSwitchState",
		// this.lowSwitch().isPressed());

		// SmartDashboard.putBoolean("intake/hasNote", this.hasNote());

		log("intakeDutyCycle", intakeMotor.get());
		log("hasNote", this.hasNote());
		log("noteTooHigh", this.noteTooHigh());
		log("lowerSensoDistance", this.lowerSensor.getRange());
		log("upperSensor", this.upperSensor.getRange());
	}

	public void set(double speed) {
		intakeMotor.set(-speed);
	}

	public TimeOfFlight lowerSensor() {
		return this.lowerSensor;
	}

	public TimeOfFlight upperSensor() {
		return this.upperSensor;
	}

	public SparkLimitSwitch lowSwitch() {
		return this.intakeMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
	}

	public SparkLimitSwitch highSwitch() {
		return this.intakeMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
	}

	public boolean hasNote() {
		return this.lowerSensor.getRange() < 270;
	}

	public boolean noteTooHigh() {
		return this.upperSensor.getRange() < 320;
	}

	private DIOSim intakeSwitchSim;
	private DCMotorSim intakeSim;
	private RevEncoderSimWrapper intakeEncSim;
	private SimTimeOfFlight lowerSim;
	private SimTimeOfFlight upperSim;
	private Debouncer intakeDebounceSim = new Debouncer(0.2);
	public void simInit() {
		intakeSwitchSim = new DIOSim(Constants.DIO.kIntakeSwitch);
		intakeSim = new DCMotorSim(DCMotor.getNEO(1), 1, 0.3);
		intakeEncSim = RevEncoderSimWrapper.create(intakeMotor);
		lowerSim = new SimTimeOfFlight(lowerSensor);
		upperSim = new SimTimeOfFlight(upperSensor);
		upperSim.setRange(400);
	}

	@Override
	public void simulationPeriodic() {
		double intakeVolts = MathUtil.clamp(intakeMotor.get() * 12, -12, 12);
		intakeSim.setInput(intakeVolts);
		Robot.objSim.setIntake(intakeDebounceSim.calculate(intakeMotor.get() < 0));

		intakeSim.update(Constants.ksimDtSec);
		if (Robot.objSim.hasObject()) {
			lowerSim.setRange(20);
		} else {
			lowerSim.setRange(400);
		}
		intakeSwitchSim.setValue(true); // TODO placeholder.
		intakeEncSim.setDistance(intakeSim.getAngularPositionRotations());
		intakeEncSim.setVelocity(intakeSim.getAngularVelocityRPM());

	}
}
