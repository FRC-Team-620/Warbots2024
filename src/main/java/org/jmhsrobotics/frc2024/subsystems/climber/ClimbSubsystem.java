package org.jmhsrobotics.frc2024.subsystems.climber;

import org.jmhsrobotics.frc2024.Constants;
import org.jmhsrobotics.warcore.rev.RevEncoderSimWrapper;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {

	private CANSparkMax climbMotor = new CANSparkMax(Constants.CAN.kClimberId, MotorType.kBrushless);
	private RelativeEncoder climbEncoder = climbMotor.getEncoder();

	public ClimbSubsystem() {
		this.climbEncoder.setPosition(0);
		this.climbMotor.setIdleMode(IdleMode.kBrake);
		this.climbMotor.setSmartCurrentLimit(40);
		if (RobotBase.isSimulation()) {
			initSim();
		}

	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("climber/position", this.getClimbPosition());
	}

	public CANSparkMax getMotor() {
		return this.climbMotor;
	}

	public void setMotor(double amount) {
		this.climbMotor.set(amount);
	}

	public double getClimbPosition() {
		return this.climbEncoder.getPosition();
	}

	public void setEncoder(double amount) {
		this.climbEncoder.setPosition(amount);
	}

	public RelativeEncoder getEncoder() {
		return this.climbEncoder;
	}

	// private ElevatorSim climbSim;
	private DCMotorSim climbSim;
	private RevEncoderSimWrapper encSim;
	private void initSim() {
		climbSim = new DCMotorSim(DCMotor.getNEO(1), 300, 1); // TODO beter sim use elevator
		encSim = RevEncoderSimWrapper.create(climbMotor);
		// climbSim = new ElevatorSim(DCMotor.getNEO(1), Units.lbsToKilograms(120), 0.2,
		// 0.2,Units.feetToMeters(4),);

	}
	@Override
	public void simulationPeriodic() {
		double climberVolts = climbMotor.get() * 12;
		climbSim.setInputVoltage(climberVolts);
		climbSim.update(0.2);
		encSim.setDistance(climbSim.getAngularPositionRotations());
		encSim.setVelocity(climbSim.getAngularVelocityRPM());
	}

}
