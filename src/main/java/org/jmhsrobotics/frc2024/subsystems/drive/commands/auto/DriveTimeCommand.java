package org.jmhsrobotics.frc2024.subsystems.drive.commands.auto;

import org.jmhsrobotics.frc2024.subsystems.drive.DriveSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveTimeCommand extends Command {
    
	private DriveSubsystem drive;

	private double driveSeconds;
	private double drivePower;

	private Timer timer = new Timer();

	public DriveTimeCommand(double seconds, double power, DriveSubsystem subsystem) {

		driveSeconds = seconds;
		drivePower = power;

		drive = subsystem;

	}

	public void initialize() {

		timer.start();
		timer.reset();

	}

	@Override
	public void execute() {

		this.drive.drive(drivePower, 0, 0, true, false);

	}

	@Override
	public boolean isFinished() {

		return timer.get() >= driveSeconds;

	}

	public void end()
    {
        if (!isFinsihed())
        {
            drivePower = 0;
        }

    }

}
