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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CoralShooterInWithAutoStopCmd;
import frc.robot.commands.SwerveControlCmd;
import frc.robot.drivebase.SwerveDrive;
import frc.robot.lib.PowerDistribution;
import frc.robot.subsystems.CoralShooterSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {
  private final CoralShooterSubsystem coralShooterSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final SendableChooser<Command> autoChooser;
  private final SwerveDrive swerveDrive;
  private final SwerveControlCmd swerveJoystickCmd;
  private final CommandXboxController elevatorController;
  private final CommandXboxController mainController;
  private final PowerDistribution powerDistribution;

  private final Command intakeCommand;

  public RobotContainer() {
    powerDistribution = new PowerDistribution();
    coralShooterSubsystem = new CoralShooterSubsystem(powerDistribution);
    elevatorSubsystem = new ElevatorSubsystem();
    swerveDrive = new SwerveDrive();
    mainController = new CommandXboxController(0);
    elevatorController = new CommandXboxController(1);
    swerveJoystickCmd = new SwerveControlCmd(swerveDrive, mainController);

    intakeCommand = new SequentialCommandGroup(
        coralShooterSubsystem.coralShooterFastOnCmd().until(coralShooterSubsystem::isGetTarget),
        new CoralShooterInWithAutoStopCmd(coralShooterSubsystem));

    NamedCommands.registerCommand("coralShooterSlowOn",
        coralShooterSubsystem.coralShooterSlowOnCmd());
    NamedCommands.registerCommand("toSecFloorCmd",
        elevatorSubsystem.toSecFloorCmd());

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("DoNothing", Commands.none());
    SmartDashboard.putData("CoralShooterSubsystem", coralShooterSubsystem);
    SmartDashboard.putData("AutoChooser", autoChooser);
    configureBindings();
  }

  private void configureBindings() {
    swerveDrive.setDefaultCommand(swerveJoystickCmd);
    mainController.a().whileTrue(swerveDrive.setTurningDegreeCmd(90));
    mainController.b().whileTrue(swerveDrive.setTurningDegreeCmd(0));
    mainController.back().onTrue(swerveDrive.gyroResetCmd());

    mainController.pov(0).whileTrue(intakeCommand);

    elevatorController.a().whileTrue(elevatorSubsystem.toSecFloorCmd());
    elevatorController.b().whileTrue(elevatorSubsystem.toTrdFloorCmd());
    elevatorController.x().whileTrue(elevatorSubsystem.toTopFloorCmd());
    elevatorController.pov(0).whileTrue(elevatorSubsystem.moveUpCmd());
    elevatorController.pov(180).whileTrue(elevatorSubsystem.moveDownCmd());

    elevatorController.rightBumper().onTrue(elevatorSubsystem.switchManualControlCmd());

    elevatorController.pov(0).whileTrue(
        Commands.either(
            elevatorSubsystem.moveUpCmd(),
            elevatorSubsystem.manualMoveCmd(0.5),
            () -> !elevatorSubsystem.isManualControl()));

    elevatorController.pov(180).whileTrue(
        Commands.either(
            elevatorSubsystem.moveDownCmd(),
            elevatorSubsystem.manualMoveCmd(-0.5),
            () -> !elevatorSubsystem.isManualControl()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

}
