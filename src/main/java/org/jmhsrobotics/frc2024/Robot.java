// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jmhsrobotics.frc2024;

import java.util.List;

import org.jmhsrobotics.frc2024.subsystems.drive.commands.IntakeCommand;
import org.jmhsrobotics.warcore.util.BuildDataLogger;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
	private Command autonomousCommand;

	private RobotContainer m_robotContainer;

	@Override
	public void robotInit() {
		m_robotContainer = new RobotContainer();
		SmartDashboard.putData("IntakeCommand", new IntakeCommand(this.m_robotContainer.getDriveSubsystem(), 5));
		setupLogs();
	}

	public void setupLogs() {
		if (Robot.isSimulation()) {
			// Filesystem.getOperatingDirectory().getAbsolutePath().
			DataLogManager.start(Filesystem.getOperatingDirectory().getAbsolutePath() + "\\logs");
		} else {
			DataLogManager.start();
		}
		DataLogManager.logNetworkTables(true);
		DriverStation.startDataLog(DataLogManager.getLog());
		BuildDataLogger.LogToNetworkTables(BuildConstants.class);
		BuildDataLogger.LogToWpiLib(DataLogManager.getLog(), BuildConstants.class);
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void disabledExit() {
	}

	@Override
	public void autonomousInit() {
		autonomousCommand = m_robotContainer.getAutonomousCommand();

		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}

		List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
				new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)), new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
				new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(0)));

		// Create the path using the bezier points created above
		PathPlannerPath path = new PathPlannerPath(bezierPoints,
				new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this path. If using a
																			// differential drivetrain, the angular
																			// constraints
																			// have no effect.
				new GoalEndState(0.0, Rotation2d.fromDegrees(0)) // Goal end state. You can set a holonomic rotation
																	// here. If
																	// using a differential drivetrain, the rotation
																	// will have no
																	// effect.
		);

	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {     

		}



	}

	@Override
	public void teleopExit() {
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void testExit() {
	}

	@Override
	public void simulationInit() {
		DriverStationSim.setEnabled(true);
	}
}
