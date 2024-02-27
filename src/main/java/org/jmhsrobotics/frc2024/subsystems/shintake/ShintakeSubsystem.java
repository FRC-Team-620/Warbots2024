// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jmhsrobotics.frc2024.subsystems.shintake;

import org.jmhsrobotics.frc2024.Constants;
import org.jmhsrobotics.frc2024.Constants.Shintake;
import org.jmhsrobotics.warcore.rev.RevEncoderSimWrapper;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShintakeSubsystem extends SubsystemBase {
	/* ---------------- INTAKE ---------------- */
	private CANSparkMax intakeMotor;
	private TimeOfFlight lowerSensor;
	private TimeOfFlight upperSensor;

	/* ---------------- SHOOTER ---------------- */
	private CANSparkMax topFlywheel;
	private CANSparkMax bottomFlywheel;
	private RelativeEncoder topEncoder;
	private RelativeEncoder bottomEncoder;

	private BangBangController bangBangController;
	private double reference;
	private double tolerance = 100;
	private ShooterControlType controlType = ShooterControlType.BANG_BANG;

	public enum ShooterControlType {
		BANG_BANG, VOLTAGE
	};

	/** Creates a new ShintakeSubsystem. */
	public ShintakeSubsystem() {
		initIntake();
		initShooter();
		if (RobotBase.isSimulation()) {
			initSim();
		}
	}

	// initialize intake motors and sensors
	private void initIntake() {
		intakeMotor = new CANSparkMax(Constants.CAN.kIntakeId, MotorType.kBrushless);
		intakeMotor.setInverted(true);
		intakeMotor.setIdleMode(IdleMode.kBrake);

		this.lowerSensor = new TimeOfFlight(1);
		this.upperSensor = new TimeOfFlight(0);
		intakeMotor.setSmartCurrentLimit(35);

		this.lowerSensor.setRangingMode(RangingMode.Short, 24);
		this.upperSensor.setRangingMode(RangingMode.Short, 24);
	}

	// initialize shooter motors and sensors
	private void initShooter() {
		topFlywheel = new CANSparkMax(Constants.CAN.kShooterTopId, MotorType.kBrushless);
		bottomFlywheel = new CANSparkMax(Constants.CAN.kShooterBottomId, MotorType.kBrushless);

		this.topFlywheel.setIdleMode(IdleMode.kCoast);
		this.topFlywheel.setSmartCurrentLimit(35);
		this.topFlywheel.setOpenLoopRampRate(.5);
		this.topEncoder = topFlywheel.getEncoder();

		this.bottomFlywheel.setIdleMode(IdleMode.kCoast);
		this.bottomFlywheel.setSmartCurrentLimit(35);
		this.bottomFlywheel.setOpenLoopRampRate(.5);
		this.bottomEncoder = bottomFlywheel.getEncoder();

		// no inversion necessary because the motors face opposing ways
		this.bottomFlywheel.follow(topFlywheel);

		SmartDashboard.putNumber("shooter/goal", 0);
		SmartDashboard.putNumber("shooter/volt", 5.5);

		this.bangBangController = new BangBangController();
		this.bangBangController.setTolerance(this.tolerance);
	}

	/* ---------------- INTAKE ---------------- */
	public void setIntakeSpeed(double speed) {
		intakeMotor.set(-speed);
	}

	public TimeOfFlight lowerIntakeSensor() {
		return this.lowerSensor;
	}

	public TimeOfFlight upperIntakeSensor() {
		return this.upperSensor;
	}

	// note in system
	public boolean hasNote() {
		return this.lowerSensor.getRange() < Shintake.hasNoteDistance;
	}

	// note is in the shooter wheels
	public boolean noteTooHigh() {
		return this.upperSensor.getRange() < Shintake.noNoteDistance;
	}

	// no note in system
	public boolean hasNoNote() {
		return this.lowerSensor.getRange() >= Shintake.noNoteDistance
				&& this.upperSensor.getRange() < Shintake.noNoteDistance;
	}

	public void fire() {
		setIntakeSpeed(Shintake.intakeOrShootSpeed);
	}

	public void intake() {
		setIntakeSpeed(Shintake.intakeOrShootSpeed);
	}

	public void ejectNote() {
		setIntakeSpeed(Shintake.ejectSpeed);
	}

	public void stopIntake() {
		setIntakeSpeed(0);
	}

	/* ---------------- SHOOTER ---------------- */
	public double getShooterRPM() {
		return topEncoder.getVelocity();
	}

	public boolean inSpinup() {
		return this.bangBangController.getError() > this.tolerance;
	}

	public void setShooterSpeed(double goal, ShooterControlType controlType) {
		this.reference = goal;
		this.controlType = controlType;
	}

	public void spinupShooter() {
		setShooterSpeed(Shintake.shootRPM, ShooterControlType.BANG_BANG);
	}

	public void stopShooter() {
		setShooterSpeed(0, ShooterControlType.BANG_BANG);
	}

	public void idleShooterForintake() {
		setShooterSpeed(-.05, ShooterControlType.VOLTAGE);

	}

	public boolean shooterSpeedAtGoal() {
		return this.bangBangController.atSetpoint();
	}

	private void shooterPeriodicTasks() {
		switch (this.controlType) {
			case BANG_BANG :
				this.topFlywheel.set(this.bangBangController.calculate(this.getShooterRPM(), this.reference));
				break;

			case VOLTAGE :
				this.topFlywheel.setVoltage(this.reference);
				break;
		}
	}

	private void intakePeriodicTasks() {
	}

	@Override
	public void periodic() {
		shooterPeriodicTasks();
		publishOdom();
	}

	private void publishOdom() {
		// intake
		SmartDashboard.putNumber("intake/velocityRPM", intakeMotor.getEncoder().getVelocity());
		SmartDashboard.putNumber("intake/currentDrawAmps", intakeMotor.getOutputCurrent());
		SmartDashboard.putNumber("Intake/lowerSensorReading", this.lowerSensor.getRange());
		SmartDashboard.putNumber("Intake/upperSensorReading", this.upperSensor.getRange());
		SmartDashboard.putBoolean("intake/hasNote", this.hasNote());

		// shooter

	}

	/* ---------------- SIM ---------------- */
	// intake
	private DIOSim intakeSwitchSim;
	private DCMotorSim intakeSim;
	private RevEncoderSimWrapper intakeEncSim;

	// shooter
	private FlywheelSim flywheelSim;
	private RevEncoderSimWrapper encSim;

	public void initSim() {
		intakeSwitchSim = new DIOSim(Constants.DIO.kIntakeSwitch);
		intakeSim = new DCMotorSim(DCMotor.getNEO(1), 1, 0.3);
		intakeEncSim = RevEncoderSimWrapper.create(intakeMotor);

		flywheelSim = new FlywheelSim(DCMotor.getNEO(1), 1, 1);
		encSim = RevEncoderSimWrapper.create(topFlywheel);
	}

	@Override
	public void simulationPeriodic() {
		// intake
		double intakeVolts = MathUtil.clamp(intakeMotor.get() * 12, -12, 12);
		intakeSim.setInput(intakeVolts);
		intakeSim.update(Constants.ksimDtSec);
		intakeSwitchSim.setValue(true); // TODO placeholder.
		intakeEncSim.setDistance(intakeSim.getAngularPositionRotations());
		intakeEncSim.setVelocity(intakeSim.getAngularVelocityRPM());

		// shooter
		double motorVolts = MathUtil.clamp(topFlywheel.get() * 12, -12, 12);
		flywheelSim.setInputVoltage(motorVolts);
		flywheelSim.update(Constants.ksimDtSec);
		encSim.setVelocity(flywheelSim.getAngularVelocityRPM());
	}
}
