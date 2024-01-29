package org.jmhsrobotics.frc2024.subsystems.shooter;

import org.jmhsrobotics.frc2024.Constants;
import org.jmhsrobotics.warcore.rev.RevEncoderSimWrapper;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

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
		topFlywheel.setIdleMode(IdleMode.kCoast);
		topFlywheel.setSmartCurrentLimit(20);
		topFlywheel.setOpenLoopRampRate(20);
		topEncoder = topFlywheel.getEncoder();

		bottomFlywheel.setIdleMode(IdleMode.kCoast);
		bottomFlywheel.setSmartCurrentLimit(20);
		bottomFlywheel.setOpenLoopRampRate(20);
		bottomEncoder = bottomFlywheel.getEncoder();

		bottomFlywheel.follow(topFlywheel);
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
		topFlywheel.set(speed);
	}

	FlywheelSim flywheelSim;
	RevEncoderSimWrapper encSim;
	public void initSim() {
		flywheelSim = new FlywheelSim(DCMotor.getNEO(1), 1, 1);
		encSim = RevEncoderSimWrapper.create(topFlywheel);
	}

	@Override
	public void simulationPeriodic() {
		double motorVolts = topFlywheel.get() * 12;
		flywheelSim.setInputVoltage(motorVolts);
		flywheelSim.update(0.2);
		encSim.setVelocity(flywheelSim.getAngularVelocityRPM());
	}
}
