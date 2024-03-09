package org.jmhsrobotics.frc2024.subsystems.intake;

import org.jmhsrobotics.frc2024.Constants;
import org.jmhsrobotics.warcore.rev.RevEncoderSimWrapper;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import monologue.Logged;

public class IntakeSubsystem extends SubsystemBase implements Logged {
	private CANSparkMax intakeMotor;

	private TimeOfFlight lowerSensor;
	private TimeOfFlight upperSensor;

	public SysIdRoutine routine;

	public IntakeSubsystem() {
		intakeMotor = new CANSparkMax(Constants.CAN.kIntakeId, MotorType.kBrushless);
		intakeMotor.setInverted(true);
		intakeMotor.setIdleMode(IdleMode.kBrake);

		this.lowerSensor = new TimeOfFlight(1);
		this.upperSensor = new TimeOfFlight(0);
		intakeMotor.setSmartCurrentLimit(35);

		this.lowerSensor.setRangingMode(RangingMode.Short, 24);
		this.upperSensor.setRangingMode(RangingMode.Short, 24);

		this.routine = new SysIdRoutine(new SysIdRoutine.Config(),
				new SysIdRoutine.Mechanism(this::voltageDrive, null, this));

		if (RobotBase.isSimulation()) {
			simInit();
		}
	}

	private void voltageDrive(Measure<Voltage> num) {
		this.intakeMotor.setVoltage(num.baseUnitMagnitude());
	}

	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return this.routine.quasistatic(direction);
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return this.routine.dynamic(direction);
	}
	@Override
	public void periodic() {
		// SmartDashboard.putNumber("intake/velocityRPM",
		// intakeMotor.getEncoder().getVelocity());
		// SmartDashboard.putNumber("intake/currentDrawAmps",
		// intakeMotor.getOutputCurrent());
		// SmartDashboard.putBoolean("Intake/highSwitchState",
		// this.highSwitch().isPressed());
		// SmartDashboard.putBoolean("Intake/lowSwitchState",
		// this.lowSwitch().isPressed());

		// SmartDashboard.putNumber("Intake/lowerSensorReading",
		// this.lowerSensor.getRange());
		// SmartDashboard.putNumber("Intake/upperSensorReading",
		// this.upperSensor.getRange());

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
		return this.lowerSensor.getRange() < 100;
	}

	public boolean noteTooHigh() {
		return this.upperSensor.getRange() < 300;
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
		double intakeVolts = MathUtil.clamp(intakeMotor.get() * 12, -12, 12);
		intakeSim.setInput(intakeVolts);
		intakeSim.update(Constants.ksimDtSec);
		intakeSwitchSim.setValue(true); // TODO placeholder.
		intakeEncSim.setDistance(intakeSim.getAngularPositionRotations());
		intakeEncSim.setVelocity(intakeSim.getAngularVelocityRPM());

	}
}
