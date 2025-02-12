// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.PowerDistribution;
import frc.robot.subsystems.CoralShooterSubsystem;


public class RobotContainer {
  private final CoralShooterSubsystem coralShooterSubsystem;
  private final SendableChooser<Command> autChooser;
  private final PowerDistribution powerDistribution;

  public RobotContainer() {
    powerDistribution = new PowerDistribution();
    coralShooterSubsystem = new CoralShooterSubsystem();
    autChooser = AutoBuilder.buildAutoChooser();
    autChooser.setDefaultOption("DoNothing", Commands.none());
    SmartDashboard.putData("CoralShooterSubsystem", coralShooterSubsystem);
    SmartDashboard.putData("AutoChooser", autChooser);
    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {

    return Commands.print("No autonomous command configured");
  }
}
