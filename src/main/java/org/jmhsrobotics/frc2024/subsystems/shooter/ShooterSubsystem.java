package org.jmhsrobotics.frc2024.subsystems.shooter;

import org.jmhsrobotics.frc2024.Constants;
import org.jmhsrobotics.warcore.rev.RevEncoderSimWrapper;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;

public class ShooterSubsystem extends SubsystemBase implements Logged {
	private CANSparkMax topFlywheel = new CANSparkMax(Constants.CAN.kShooterTopId, MotorType.kBrushless);
	private CANSparkMax bottomFlywheel = new CANSparkMax(Constants.CAN.kShooterBottomId, MotorType.kBrushless);;
	private RelativeEncoder topEncoder;
	private RelativeEncoder bottomEncoder;

	private BangBangController bangBangController;
	private double reference;
	private ControlType controlType = ControlType.BANG_BANG;
	public enum ControlType {
		BANG_BANG, VOLTAGE
	};

	// private double speed;

	public ShooterSubsystem() {
		// Initializes motor(s)
		this.bangBangController = new BangBangController();
		this.bangBangController.setTolerance(100);

		initializeMotors();
		if (RobotBase.isSimulation()) {
			initSim();
		}
	}

	@Override
	public void periodic() {

		switch (this.controlType) {
			case BANG_BANG :
				this.topFlywheel.set(this.bangBangController.calculate(this.getRPM(), this.reference));
				break;

			case VOLTAGE :
				this.topFlywheel.setVoltage(this.reference);
				break;
		}
		log("controlType", this.controlType.toString());
		log("reference", this.reference);
		log("topFlywheelDutyCycle", topFlywheel.get());
		log("topflywheelSpeed", getRPM());
		log("bottomflywheelSpeed", bottomEncoder.getVelocity());

	}

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
		this.topFlywheel.setIdleMode(IdleMode.kCoast);
		this.topFlywheel.setSmartCurrentLimit(35);
		this.topFlywheel.setOpenLoopRampRate(.5);
		this.topEncoder = topFlywheel.getEncoder();

		this.bottomFlywheel.setIdleMode(IdleMode.kCoast);
		this.bottomFlywheel.setSmartCurrentLimit(35);
		this.bottomFlywheel.setOpenLoopRampRate(.5);
		this.bottomEncoder = bottomFlywheel.getEncoder();

		this.bottomFlywheel.follow(topFlywheel);
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
