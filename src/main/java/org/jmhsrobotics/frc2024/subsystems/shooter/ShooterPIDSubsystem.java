package org.jmhsrobotics.frc2024.subsystems.shooter;

import org.jmhsrobotics.frc2024.Constants;
import org.jmhsrobotics.warcore.rev.RevEncoderSimWrapper;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterPIDSubsystem extends SubsystemBase {
	private CANSparkMax topFlywheel = new CANSparkMax(Constants.CAN.kShooterTopId, MotorType.kBrushless);
	private CANSparkMax bottomFlywheel = new CANSparkMax(Constants.CAN.kShooterBottomId, MotorType.kBrushless);;
	private RelativeEncoder topEncoder;
	private RelativeEncoder bottomEncoder;
	private PIDController shooterPID;
	private double speed;

	// private double speed;

	public ShooterPIDSubsystem() {
		// Initializes motor(s)
		initializeMotors();
		if (RobotBase.isSimulation()) {
			initSim();
		}
	}

	private void initializeMotors() {
		this.topFlywheel.setIdleMode(IdleMode.kCoast);
		this.topFlywheel.setSmartCurrentLimit(20);
		this.topFlywheel.setOpenLoopRampRate(.5);
		this.topEncoder = topFlywheel.getEncoder();

		this.bottomFlywheel.setIdleMode(IdleMode.kCoast);
		this.bottomFlywheel.setSmartCurrentLimit(20);
		this.bottomFlywheel.setOpenLoopRampRate(.5);
		this.bottomEncoder = bottomFlywheel.getEncoder();

		this.bottomFlywheel.follow(topFlywheel);

		shooterPID = new PIDController(.004, 0, 0);
		shooterPID.setSetpoint(speed);
		SmartDashboard.putNumber("shooter/goal", 0);
		SmartDashboard.putNumber("shooter/P", shooterPID.getP());
		SmartDashboard.putNumber("shooter/I", shooterPID.getI());
		SmartDashboard.putNumber("shooter/D", shooterPID.getD());

	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("shooter/topRPM", getRPM());
		SmartDashboard.putNumber("shooter/bottomRPM", bottomEncoder.getVelocity());
		double PIDOut = shooterPID.calculate(getRPM());
		PIDOut = MathUtil.clamp(PIDOut, -.1, 6);
		setVolt(PIDOut);
		SmartDashboard.putNumber("shooter/PIDOut", PIDOut);
		setGoal(SmartDashboard.getNumber("shooter/goal", 0));
		shooterPID.setP(SmartDashboard.getNumber("shooter/P", 0));
		shooterPID.setI(SmartDashboard.getNumber("shooter/I", 0));
		shooterPID.setD(SmartDashboard.getNumber("shooter/D", 0));

	}

	public double getRPM() {

		return topEncoder.getVelocity();
	}

	public void setGoal(double goal) {
		shooterPID.setSetpoint(goal);
	}

	public boolean atGoal() {
		return shooterPID.atSetpoint();
	}

	public void setSpeed(double speed) {
		this.topFlywheel.set(speed);
	}

	public void setVolt(double amount) {
		this.topFlywheel.setVoltage(amount);
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
