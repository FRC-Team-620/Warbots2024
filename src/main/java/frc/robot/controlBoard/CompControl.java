package frc.robot.controlBoard;

import javax.naming.ldap.Control;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CompControl implements ControlBoard{
    private CommandXboxController driver = new CommandXboxController(0);
    private CommandXboxController operator = new CommandXboxController(1);
    
    @Override
    public Trigger brake() {
        // TODO Auto-generated method stub
        return driver.leftBumper();
    }

    @Override
    public Trigger zeroHeading() {
        // TODO Auto-generated method stub
        return driver.rightBumper();
    }
    
}
