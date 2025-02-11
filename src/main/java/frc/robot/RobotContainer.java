// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {

  ElevatorSubsystem elevatorSubsystem;
  CommandXboxController joy;

  public RobotContainer() {
    elevatorSubsystem = new ElevatorSubsystem();
    joy = new CommandXboxController(0);
   

    configureBindings();

  }

  private void configureBindings() {
    joy.pov(0).whileTrue(elevatorSubsystem.moveUpCmd());
    joy.pov(180).whileTrue(elevatorSubsystem.moveDownCmd());
    joy.pov(45).whileTrue(elevatorSubsystem.toSecFloorCmd());
    joy.pov(90).whileTrue(elevatorSubsystem.toTrdFloorCmd());
    joy.pov(135).whileTrue(elevatorSubsystem.toTopFloorCmd());
    joy.a().whileTrue(elevatorSubsystem.toDefaultPositionCmd());
    joy.b().whileTrue(elevatorSubsystem.stopMoveCmd());


  }

  public Command getAutonomousCommand() {

    return Commands.print("No autonomous command configured");
  }
}
