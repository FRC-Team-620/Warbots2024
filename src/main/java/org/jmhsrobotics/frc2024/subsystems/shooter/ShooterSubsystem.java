package org.jmhsrobotics.frc2024.subsystems.shooter;

import org.jmhsrobotics.frc2024.Constants;
import org.jmhsrobotics.warcore.rev.RevEncoderSimWrapper;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import monologue.Logged;

public class ShooterSubsystem extends SubsystemBase implements Logged {
	private CANSparkMax topFlywheel = new CANSparkMax(Constants.CAN.kShooterTopId, MotorType.kBrushless);
	private CANSparkMax bottomFlywheel = new CANSparkMax(Constants.CAN.kShooterBottomId, MotorType.kBrushless);;
	private RelativeEncoder topEncoder;
	private RelativeEncoder bottomEncoder;

	private BangBangController bangBangController;
	private double reference;
	private ControlType controlType = ControlType.BANG_BANG;

	private PIDController upperPID;
	private PIDController lowerPID;

	private SysIdRoutine routine;

	public enum ControlType {
		BANG_BANG, VOLTAGE, PID
	};

	// private double speed;

	public ShooterSubsystem() {
		// Initializes motor(s)
		this.bangBangController = new BangBangController();
		this.bangBangController.setTolerance(200);

		this.upperPID = new PIDController(0.01, 0, 0);
		this.lowerPID = new PIDController(0.01, 0, 0);
		initializeMotors();
		if (RobotBase.isSimulation()) {
			initSim();
		}

		SmartDashboard.putData("ShooterUpperPID", this.upperPID);
		SmartDashboard.putData("ShooterLowerPID", this.lowerPID);

		this.routine = new SysIdRoutine(new SysIdRoutine.Config(),
				new SysIdRoutine.Mechanism(this::voltageDrive, null, this));
	}

	private void voltageDrive(Measure<Voltage> num) {
		topFlywheel.setVoltage(num.baseUnitMagnitude());
	}

	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return routine.quasistatic(direction);
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return routine.dynamic(direction);
	}

	// @Override
	// public void periodic() {

	// switch (this.controlType) {
	// case BANG_BANG :
	// double outPut = this.bangBangController.calculate(this.getRPM(),
	// this.reference);
	// this.topFlywheel.set(outPut);
	// this.bottomFlywheel.set(outPut);
	// break;

	// case VOLTAGE :
	// this.topFlywheel.setVoltage(this.reference);
	// this.bottomFlywheel.setVoltage(this.reference);
	// break;
	// case PID :
	// double upperOutput = MathUtil
	// .clamp(this.upperPID.calculate(this.topEncoder.getVelocity(),
	// this.reference), -1, 1);
	// double lowerOutput = MathUtil
	// .clamp(this.lowerPID.calculate(this.bottomEncoder.getVelocity(),
	// this.reference), -1, 1);
	// this.topFlywheel.set(upperOutput);
	// this.bottomFlywheel.set(lowerOutput);
	// }
	// log("controlType", this.controlType.toString());
	// log("reference", this.reference);
	// log("topFlywheelDutyCycle", topFlywheel.get());
	// log("topflywheelSpeed", getRPM());
	// log("bottomflywheelSpeed", bottomEncoder.getVelocity());

	// }

	public double getRPM() {
		return topEncoder.getVelocity();
	}

	public void set(double goal, ControlType controlType) {
		this.reference = goal;
		this.controlType = controlType;
	}

	public boolean atGoal() {
		if (controlType == ControlType.VOLTAGE) {
			return false;
		} else {
			return this.bangBangController.atSetpoint();
		}

	}
	private void initializeMotors() {
		this.topFlywheel.restoreFactoryDefaults();
		this.topFlywheel.setIdleMode(IdleMode.kCoast);
		this.topFlywheel.setSmartCurrentLimit(60);
		this.topFlywheel.setOpenLoopRampRate(0.0);
		this.topEncoder = topFlywheel.getEncoder();
		this.topEncoder.setMeasurementPeriod(16);
		this.topEncoder.setAverageDepth(2);

		this.bottomFlywheel.restoreFactoryDefaults();
		this.bottomFlywheel.setIdleMode(IdleMode.kCoast);
		this.bottomFlywheel.setSmartCurrentLimit(60);
		this.bottomFlywheel.setOpenLoopRampRate(0.0);
		this.bottomEncoder = bottomFlywheel.getEncoder();
		this.bottomEncoder.setMeasurementPeriod(16);
		this.bottomEncoder.setAverageDepth(2);

		// this.bottomFlywheel.follow(topFlywheel);
	}

	FlywheelSim flywheelSim;
	RevEncoderSimWrapper encSim;
	public void initSim() {
		flywheelSim = new FlywheelSim(DCMotor.getNEO(1), 1, 1);
		encSim = RevEncoderSimWrapper.create(topFlywheel);
	}

	@Override
	public void simulationPeriodic() {
		double motorVolts = MathUtil.clamp(topFlywheel.get() * 12, -12, 12);
		flywheelSim.setInputVoltage(motorVolts);
		flywheelSim.update(Constants.ksimDtSec);
		encSim.setVelocity(flywheelSim.getAngularVelocityRPM());
	}
}
