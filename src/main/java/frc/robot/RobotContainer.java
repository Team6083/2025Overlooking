// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.ColorSensorV3.MainControl;

// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CoralShooterSubsystem;
import frc.robot.drivebase.SwerveDrive;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {
  private final CoralShooterSubsystem coralShooterSubsystem;
  // private final SendableChooser<Command> autChooser;
  private final SwerveDrive swerveDrive;
  private final CommandXboxController mainController;

  public RobotContainer() {
    coralShooterSubsystem = new CoralShooterSubsystem();
    swerveDrive = new SwerveDrive();
    mainController = new CommandXboxController(0);
    // autChooser = AutoBuilder.buildAutoChooser();
    // autChooser.setDefaultOption("DoNothing", Commands.none());
    SmartDashboard.putData("CoralShooterSubsystem", coralShooterSubsystem);
    // SmartDashboard.putData("AutoChooser", autChooser);
    configureBindings();
  }

  private void configureBindings() {
    mainController.leftBumper().onTrue(swerveDrive.TagTrackingCmd());

    
  }

  public Command getAutonomousCommand() {

    return Commands.print("No autonomous command configured");
  }
}
