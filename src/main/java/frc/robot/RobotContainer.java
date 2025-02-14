// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.CoralShooterInWithAutoStopCmd;
import frc.robot.subsystems.CoralShooterSubsystem;

public class RobotContainer {
  private final CoralShooterSubsystem coralShooterSubsystem;
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    coralShooterSubsystem = new CoralShooterSubsystem();

    NamedCommands.registerCommand("CoralShooterInWithAutoStopCmd",
        new CoralShooterInWithAutoStopCmd(coralShooterSubsystem));

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("DoNothing", Commands.none());
    SmartDashboard.putData("CoralShooterSubsystem", coralShooterSubsystem);
    SmartDashboard.putData("AutoChooser", autoChooser);
    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {

    return Commands.print("No autonomous command configured");
  }
}
