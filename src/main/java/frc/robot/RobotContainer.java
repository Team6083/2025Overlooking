// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.drivebase.SwerveDrive;
// import frc.robot.subsystems.AlgaeIntakeSubsystem;
// import frc.robot.subsystems.ClimberSubsystem;
// import frc.robot.subsystems.CoralShooterSubsystem;

public class RobotContainer {
  // private final ClimberSubsystem climberSubsystem;
  // private final AlgaeIntakeSubsystem algaeIntakeSubsystem;
  // private final CoralShooterSubsystem coralShooterSubsystem;
  // private final SendableChooser<Command> autChooser;
  private final SwerveDrive swerveDrive;
  private final SwerveJoystickCmd swerveJoystickCmd;
  private final CommandXboxController mainController;

  public RobotContainer() {
    // coralShooterSubsystem = new CoralShooterSubsystem();
    // climberSubsystem = new ClimberSubsystem();
    // algaeIntakeSubsystem = new AlgaeIntakeSubsystem();
    swerveDrive = new SwerveDrive();
    mainController = new CommandXboxController(0);
    swerveJoystickCmd = new SwerveJoystickCmd(swerveDrive, mainController);
    // autChooser = AutoBuilder.buildAutoChooser();
    // autChooser.setDefaultOption("DoNothing", Commands.none());
    // SmartDashboard.putData("CoralShooterSubsystem", coralShooterSubsystem);
    // SmartDashboard.putData("AutoChooser", autChooser);
    // SmartDashboard.putData("AlgaeIntakeSubsystem", algaeIntakeSubsystem);
    // SmartDashboard.putData("ClimberSubsystem", climberSubsystem);

    configureBindings();
  }

  private void configureBindings() {
    swerveDrive.setDefaultCommand(swerveJoystickCmd);
    mainController.a().whileTrue(swerveDrive.setTurningDegreeCmd(90));
    mainController.b().whileTrue(swerveDrive.setTurningDegreeCmd(0));

    mainController.y().whileTrue(
      swerveDrive.runEnd(
        ()->swerveDrive.drive(-0.5,0.5,0,true),
        ()->swerveDrive.stop()));
    // mainController.x().whileTrue(
    //   swerveDrive.runEnd(
    //     ()->swerveDrive.drive(0,0.1,0,false), 
    //     ()->swerveDrive.stop()));

    // mainController.x().whileTrue(
    //   swerveDrive.runEnd(
    //     ()-> swerveDrive.frontLeft.setDesiredState(
    //       new SwerveModuleState(0.5, new Rotation2d(1))),
    //     ()-> swerveDrive.frontLeft.stopModule()));

    mainController.back().whileTrue(swerveDrive.gyroResetCmd());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
