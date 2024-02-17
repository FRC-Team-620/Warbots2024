package org.jmhsrobotics.frc2024.controlBoard;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CompControl implements ControlBoard {
    private XboxController driver = new XboxController(0);
    private XboxController operator = new XboxController(1);

	// =============Driver Controls=============
	@Override
	public double xInput() {
		return this.driver.getLeftX();
	}

    // =============Driver Controls=============
    @Override
    public double xInput() {

        return this.driver.getLeftX();
    }

    @Override
    public double yInput() {
        return this.driver.getLeftY();
    }

    @Override
    public double rotationalInput() {
        return this.driver.getRightX();
    }

    @Override
    public boolean brake() {
        return this.driver.getLeftBumper();
    }

    @Override
    public boolean setZeroHeading() {
        return this.driver.getRightBumper();
    }

    // =============Utils=============
    @Override
    public XboxController getDriverController() {
        return this.driver;
    }

    @Override
    public XboxController getOperatorController() {
        return this.operator;
    }
	@Override
	public double yInput() {
		return this.driver.getLeftY();
	}

	@Override
	public double rotationalInput() {
		return this.driver.getRightX();
	}

	@Override
	public Trigger brake() {
		return this.driver.leftBumper();
	}

	@Override
	public Trigger setZeroHeading() {
		return this.driver.rightBumper();
	}

	// =============Operator Controls=============
	public double pitchInput() {
		return this.operator.getRightY();
	}

	public Trigger presetHigh() {
		return this.operator.y();
	}

	@Override
	public Trigger presetMid() {
		return this.operator.b();
	}

	public Trigger presetLow() {
		return this.operator.a();
	}

	@Override
	public Trigger intakeInput() {
		return this.operator.rightTrigger();
	}

	@Override
	public Trigger extakeInput() {
		return this.operator.leftTrigger();
	}

	@Override
	public Trigger shooterInput() {
		return this.operator.rightBumper();
	}

	@Override
	public Trigger climberExtend() {
		return this.operator.povUp();
	}

	@Override
	public Trigger climberRetract() {
		return this.operator.povDown();
	}
}
