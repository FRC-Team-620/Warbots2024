package org.jmhsrobotics.frc2024.subsystems.shooter;

import org.jmhsrobotics.frc2024.Constants;
import org.jmhsrobotics.warcore.rev.RevEncoderSimWrapper;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
	private CANSparkMax topFlywheel = new CANSparkMax(Constants.CAN.kShooterTopId, MotorType.kBrushless);
	private CANSparkMax bottomFlywheel = new CANSparkMax(Constants.CAN.kShooterBottomId, MotorType.kBrushless);;
	private RelativeEncoder topEncoder;
	private RelativeEncoder bottomEncoder;

	// private double speed;

	public ShooterSubsystem() {
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
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("shooter/topRPM", getRPM());
		SmartDashboard.putNumber("shooter/bottomRPM", bottomEncoder.getVelocity());
	}

	public double getRPM() {

		return topEncoder.getVelocity();
	}

	public void setSpeed(double speed) {
		this.topFlywheel.set(speed);
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
