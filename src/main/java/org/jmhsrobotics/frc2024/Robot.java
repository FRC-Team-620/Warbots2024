// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jmhsrobotics.frc2024;

import org.jmhsrobotics.warcore.util.BuildDataLogger;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import monologue.Logged;

public class Robot extends TimedRobot implements Logged {
	private Command autonomousCommand;

	private RobotContainer m_robotContainer;

	@Override
	public void robotInit() {
		m_robotContainer = new RobotContainer();
		setupLogs();
	}

	public void setupLogs() {
		if (Robot.isSimulation()) {
			// Filesystem.getOperatingDirectory().getAbsolutePath().
			DataLogManager.start(Filesystem.getOperatingDirectory().getAbsolutePath() + "/logs");
		} else {
			DataLogManager.start();
		}
		DataLogManager.logNetworkTables(false);
		DriverStation.startDataLog(DataLogManager.getLog());
		BuildDataLogger.LogToNetworkTables(BuildConstants.class);
		BuildDataLogger.LogToWpiLib(DataLogManager.getLog(), BuildConstants.class);
		boolean fileOnly = false;
		boolean lazyLogging = false;
		// Monologue.setupMonologue(this, "Robot", fileOnly, lazyLogging);
	}

	@Override
	public void robotPeriodic() {
		// Monologue.updateAll();
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

	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
		m_robotContainer.configureTeam();
	}

	@Override
	public void teleopPeriodic() {

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
		DriverStationSim.setDsAttached(true);
		DriverStationSim.setEnabled(true);
	}
}
