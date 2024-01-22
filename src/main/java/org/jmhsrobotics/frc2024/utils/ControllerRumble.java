package org.jmhsrobotics.frc2024.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ControllerRumble {
  private GenericHID controller;

  public ControllerRumble(GenericHID controller) {
    this.controller = controller;
  }

  public void rumble(int duration, double strength) {
    for (int i = 0; i < duration; i++) {
      controller.setRumble(GenericHID.RumbleType.kLeftRumble, strength);
      controller.setRumble(GenericHID.RumbleType.kRightRumble, strength);
    }
  }
    SmartDashboard.putData(CommandScheduler.getInstance()); 

  public void disabledInit() {
    
    // add code to initialize rumble during disabled mode here
  }
}












