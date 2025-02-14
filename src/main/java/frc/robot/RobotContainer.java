// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AlgaeIntakeConstant;
import frc.robot.commands.CoralShooterInWithAutoStopCmd;
import frc.robot.commands.SwerveControlCmd;
import frc.robot.drivebase.SwerveDrive;
import frc.robot.lib.PowerDistribution;
// import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.CoralShooterSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {
  private final PowerDistribution powerDistribution;
  private final CoralShooterSubsystem coralShooterSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  // private final AlgaeIntakeSubsystem algaeIntakeSubsystem;
  // private final SendableChooser<Command> autChooser;
  private final SwerveDrive swerveDrive;
  private final SwerveControlCmd swerveJoystickCmd;
  private final CommandXboxController coController;
  private final CommandXboxController mainController;

  private final Command intakeCommand;

  public RobotContainer() {
    powerDistribution = new PowerDistribution();
    coralShooterSubsystem = new CoralShooterSubsystem(powerDistribution);
    elevatorSubsystem = new ElevatorSubsystem();
    // algaeIntakeSubsystem = new AlgaeIntakeSubsystem(powerDistribution);
    swerveDrive = new SwerveDrive();
    mainController = new CommandXboxController(0);
    coController = new CommandXboxController(1);
    swerveJoystickCmd = new SwerveControlCmd(swerveDrive, mainController);

    intakeCommand = new SequentialCommandGroup(
        coralShooterSubsystem.coralShooterFastOnCmd().until(coralShooterSubsystem::isGetTarget),
        new CoralShooterInWithAutoStopCmd(coralShooterSubsystem));

    // autChooser = AutoBuilder.buildAutoChooser();
    // autChooser.setDefaultOption("DoNothing", Commands.none());
    // SmartDashboard.putData("AutoChooser", autChooser);
    SmartDashboard.putData("CoralShooterSubsystem", coralShooterSubsystem);
    SmartDashboard.putData("ElevatorSubsystem", elevatorSubsystem);
    // SmartDashboard.putData("AlgaeIntakeSubsystem", algaeIntakeSubsystem);
    SmartDashboard.putData("SwerveDrive", swerveDrive);
    configureBindings();
  }

  private void configureBindings() {
    swerveDrive.setDefaultCommand(swerveJoystickCmd);
    mainController.back().onTrue(swerveDrive.gyroResetCmd());

    // mainController.x().onTrue(intakeCommand);
    // mainController.y().onTrue(coralShooterSubsystem.coralShooterStopCmd());
    mainController.rightBumper().whileTrue(coralShooterSubsystem.coralShooterSlowOnCmd());

    coController.y().whileTrue(elevatorSubsystem.toDefaultPositionCmd());
    coController.a().whileTrue(elevatorSubsystem.toSecFloorCmd());
    coController.b().whileTrue(elevatorSubsystem.toTrdFloorCmd());
    coController.x().whileTrue(elevatorSubsystem.toTopFloorCmd());

    mainController.a().onTrue(elevatorSubsystem.switchManualControlCmd(true));
    mainController.b().onTrue(elevatorSubsystem.switchManualControlCmd(false));

    mainController.pov(0).whileTrue(
        Commands.either(
            elevatorSubsystem.moveUpCmd(),
            elevatorSubsystem.manualMoveCmd(0.5),
            () -> !elevatorSubsystem.isManualControl()));

    mainController.pov(180).whileTrue(
        Commands.either(
            elevatorSubsystem.moveDownCmd(),
            elevatorSubsystem.manualMoveCmd(-0.5),
            () -> !elevatorSubsystem.isManualControl()));

    mainController.start().onTrue(elevatorSubsystem.elevatorReset());
    // mainController.leftTrigger().whileTrue(
    //     algaeIntakeSubsystem.intakeCmd(
    //         mainController.getLeftTriggerAxis()));
    // mainController.leftBumper().whileTrue(algaeIntakeSubsystem.reIntakeCmd());
    // mainController.povLeft().whileTrue(algaeIntakeSubsystem.downRotatePIDCmd());
    // mainController.povRight().whileTrue(algaeIntakeSubsystem.upRotatePIDCmd());

    // 測完之後希望可以做到
    // mainController.leftTrigger().onTrue(algaeIntakeSubsystem.autoIntakeCmd()
    // .alongWith(algaeIntakeSubsystem.downRotatePIDCmd()));
    // mainController.leftBumper()
    //     .whileTrue(algaeIntakeSubsystem.reIntakeCmd()
    //         .alongWith(algaeIntakeSubsystem.upRotatePIDCmd()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

}
