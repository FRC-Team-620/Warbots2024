package org.jmhsrobotics.frc2024.subsystems.shintake;

import org.jmhsrobotics.frc2024.Constants;
import org.jmhsrobotics.frc2024.Robot;
import org.jmhsrobotics.frc2024.utils.SimTimeOfFlight;
import org.jmhsrobotics.frc2024.utils.SimableTimeOfFlight;
import org.jmhsrobotics.warcore.rev.RevEncoderSimWrapper;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;

public class ShintakeSubsystem extends SubsystemBase implements Logged {
	private CANSparkMax topFlywheel = new CANSparkMax(Constants.CAN.kShooterTopId, MotorType.kBrushless);
	private CANSparkMax bottomFlywheel = new CANSparkMax(Constants.CAN.kShooterBottomId, MotorType.kBrushless);
	private CANSparkMax intakeMotor = new CANSparkMax(Constants.CAN.kIntakeId, MotorType.kBrushless);

	private RelativeEncoder topEncoder;
	private RelativeEncoder bottomEncoder;

	private BangBangController bangBangController;
	private double reference;
	private ControlType controlType = ControlType.BANG_BANG;

	private PIDController upperPID;
	private PIDController lowerPID;
	private SimpleMotorFeedforward upperFeedforward = new SimpleMotorFeedforward(0.10948, 0.019707, 0.0018555);
	private SimpleMotorFeedforward lowerFeedforward = new SimpleMotorFeedforward(0.10948, 0.019707, 0.0018555);

	private SimableTimeOfFlight lowerSensor;
	private SimableTimeOfFlight upperSensor;
	public enum ControlType {
		BANG_BANG, VOLTAGE, PID
	};

	// private double speed;

	public ShintakeSubsystem() {
		// Initializes motor(s)
		this.bangBangController = new BangBangController();
		this.bangBangController.setTolerance(200);

		this.upperPID = new PIDController(0.001, 0, 0);
		this.lowerPID = new PIDController(0.001, 0, 0);
		this.upperPID.setTolerance(Units.rotationsPerMinuteToRadiansPerSecond(100)); // WARNING: this value is in Rad/s
		this.lowerPID.setTolerance(Units.rotationsPerMinuteToRadiansPerSecond(100)); // WARNING: this value is in Rad/s

		this.lowerSensor = new SimableTimeOfFlight(1);
		this.upperSensor = new SimableTimeOfFlight(0);

		this.lowerSensor.setRangingMode(RangingMode.Short, 24);
		this.upperSensor.setRangingMode(RangingMode.Short, 24);
		initializeMotors();
		if (RobotBase.isSimulation()) {
			initSim();
		}

		// SmartDashboard.putData("ShooterUpperPID", this.upperPID);
		// SmartDashboard.putData("ShooterLowerPID", this.lowerPID);
	}

	@Override
	public void periodic() {
		switch (this.controlType) {
			case BANG_BANG :
				double outPut = this.bangBangController.calculate(this.getShooterRPM(), this.reference);
				this.topFlywheel.set(outPut);
				this.bottomFlywheel.set(outPut);
				break;

			case VOLTAGE :
				this.setVoltages(this.reference, this.reference);
				break;
			case PID :
				double upperOutput = MathUtil
						.clamp(this.upperPID.calculate(Units.rotationsPerMinuteToRadiansPerSecond(this.getTopRRPM()),
								Units.rotationsPerMinuteToRadiansPerSecond(this.reference)), -12, 12);
				double lowerOutput = MathUtil
						.clamp(this.lowerPID.calculate(Units.rotationsPerMinuteToRadiansPerSecond(this.getBottomRRPM()),
								Units.rotationsPerMinuteToRadiansPerSecond(this.reference)), -12, 12);

				upperOutput += upperFeedforward.calculate(Units.rotationsPerMinuteToRadiansPerSecond(this.reference));
				lowerOutput += this.lowerFeedforward
						.calculate(Units.rotationsPerMinuteToRadiansPerSecond(this.reference));
				this.setVoltages(upperOutput, lowerOutput);
				break;

		}
		log("controlType", this.controlType.toString());
		log("reference", this.reference);
		log("topFlywheelDutyCycle", topFlywheel.get());
		log("topflywheelSpeed", getShooterRPM());
		log("bottomflywheelSpeed", getBottomRRPM());
		log("AtGoal", this.isShooterAtGoal());

	}

	public double getShooterRPM() {
		return getTopRRPM();
	}

	public void setShooterGoal(double goal, ControlType controlType) {
		this.reference = goal;
		this.controlType = controlType;
	}

	public boolean isShooterAtGoal() {
		if (controlType == ControlType.VOLTAGE) {
			return false;
		} else {
			return this.upperPID.atSetpoint() && this.lowerPID.atSetpoint();
		}
	}

	public boolean hasNote() {
		return this.lowerSensor.getRange() < 270;
	}

	public boolean noteTooHigh() {
		return this.upperSensor.getRange() < 320;
	}

	public boolean shooterAtGoal(){
		return this.lowerPID.atSetpoint() || this.upperPID.atSetpoint();
	}
	private void initializeMotors() {
		// this.topFlywheel.restoreFactoryDefaults();
		this.topFlywheel.setIdleMode(IdleMode.kCoast);
		this.topFlywheel.setSmartCurrentLimit(60);
		this.topFlywheel.setOpenLoopRampRate(0);
		this.topEncoder = topFlywheel.getEncoder();
		this.topEncoder.setAverageDepth(2);
		this.topEncoder.setMeasurementPeriod(16);

		// this.bottomFlywheel.restoreFactoryDefaults();
		this.bottomFlywheel.setIdleMode(IdleMode.kCoast);
		this.bottomFlywheel.setSmartCurrentLimit(60);
		this.bottomFlywheel.setOpenLoopRampRate(0);
		this.bottomEncoder = bottomFlywheel.getEncoder();
		this.bottomEncoder.setAverageDepth(2);
		this.bottomEncoder.setMeasurementPeriod(16);

		intakeMotor.setInverted(true);
		intakeMotor.setIdleMode(IdleMode.kBrake);
	}

	private void setVoltages(double topVoltage, double bottomVoltage) {
		if (Robot.isSimulation()) { // TODO: Simplation Hack
			this.topFlywheel.set(topVoltage / 12.0);
			this.bottomFlywheel.set(bottomVoltage / 12.0);
		}
		this.topFlywheel.setVoltage(topVoltage);
		this.bottomFlywheel.setVoltage(bottomVoltage);
	}

	private double getBottomRRPM() {
		if (Robot.isSimulation()) {
			return simVelocity;
		}
		return bottomEncoder.getVelocity();
	}

	public void setIntakeSpeed(double speed) {
		intakeMotor.set(-speed);
	}

	private double getTopRRPM() {
		if (Robot.isSimulation()) {
			return simVelocity;
		}
		return topEncoder.getVelocity();
	}

	private FlywheelSim flywheelSim;
	private RevEncoderSimWrapper encSim;
	private DIOSim intakeSwitchSim;
	private DCMotorSim intakeSim;
	private RevEncoderSimWrapper intakeEncSim;
	private SimTimeOfFlight lowerSim;
	private SimTimeOfFlight upperSim;
	private Debouncer intakeDebounceSim = new Debouncer(0.2);
	private double simVelocity = 0;

	public void initSim() {
		flywheelSim = new FlywheelSim(DCMotor.getNEO(1), 1, 0.002);
		encSim = RevEncoderSimWrapper.create(topFlywheel);
		intakeSwitchSim = new DIOSim(Constants.DIO.kIntakeSwitch);
		intakeSim = new DCMotorSim(DCMotor.getNEO(1), 1, 0.3);
		intakeEncSim = RevEncoderSimWrapper.create(intakeMotor);
		lowerSim = new SimTimeOfFlight(lowerSensor);
		upperSim = new SimTimeOfFlight(upperSensor);
		upperSim.setRange(400);
	}

	@Override
	public void simulationPeriodic() {
		double motorVolts = MathUtil.clamp(topFlywheel.get() * 12, -12, 12);

		flywheelSim.setInputVoltage(motorVolts);
		flywheelSim.update(Constants.ksimDtSec);
		simVelocity = flywheelSim.getAngularVelocityRPM();
		double intakeVolts = MathUtil.clamp(intakeMotor.get() * 12, -12, 12);
		intakeSim.setInput(intakeVolts);
		Robot.objSim.setIntake(intakeDebounceSim.calculate(intakeMotor.get() < 0));
		intakeSim.update(Constants.ksimDtSec);

		if (simVelocity > 1000) {
			Robot.objSim.fire();
		} else {
			encSim.setVelocity(simVelocity);
		}
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
