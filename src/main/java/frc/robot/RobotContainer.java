// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveControlCmd;
import frc.robot.drivebase.SwerveDrive;
import frc.robot.lib.PowerDistribution;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.CoralShooterSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {
  private final PowerDistribution powerDistribution;
  private final CoralShooterSubsystem coralShooterSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final AlgaeIntakeSubsystem algaeIntakeSubsystem;
  private final SwerveDrive swerveDrive;
  private final SwerveControlCmd swerveJoystickCmd;
  private final CommandXboxController mainController;

  public RobotContainer() {
    powerDistribution = new PowerDistribution();
    coralShooterSubsystem = new CoralShooterSubsystem(powerDistribution);
    elevatorSubsystem = new ElevatorSubsystem();
    algaeIntakeSubsystem = new AlgaeIntakeSubsystem(powerDistribution);
    swerveDrive = new SwerveDrive();
    mainController = new CommandXboxController(0);
    swerveJoystickCmd = new SwerveControlCmd(swerveDrive, mainController);

    SmartDashboard.putData("CoralShooterSubsystem", coralShooterSubsystem);
    SmartDashboard.putData("ElevatorSubsystem", elevatorSubsystem);
    SmartDashboard.putData("AlgaeIntakeSubsystem", algaeIntakeSubsystem);
    SmartDashboard.putData("SwerveDrive", swerveDrive);
    configureBindings();
  }

  private void configureBindings() {
    swerveDrive.setDefaultCommand(swerveJoystickCmd);
    mainController.back().onTrue(swerveDrive.gyroResetCmd());

    mainController.rightBumper().whileTrue(coralShooterSubsystem.coralShooterSlowOnCmd());

    mainController.povUp().whileTrue(elevatorSubsystem.toSecFloorCmd());
    mainController.povLeft().whileTrue(elevatorSubsystem.toGetCarolHeightCmd());
    mainController.povDown().whileTrue(elevatorSubsystem.toDefaultPositionCmd());

    mainController.leftTrigger()
        .whileTrue(Commands.either(
          
            elevatorSubsystem.manualMoveCmd(-0.5),
            elevatorSubsystem.moveDownCmd(),
            mainController.povRight()));
    mainController.rightTrigger()
        .whileTrue(Commands.either(
            elevatorSubsystem.manualMoveCmd(0.5),
            elevatorSubsystem.moveUpCmd(),
            mainController.povRight()));

    mainController.start().onTrue(elevatorSubsystem.elevatorReset());
    mainController.y().whileTrue(algaeIntakeSubsystem.manualUpRotateCmd());
    mainController.a().whileTrue(algaeIntakeSubsystem.manualDownRotateCmd());
    mainController.x().whileTrue(algaeIntakeSubsystem.setIntakeMotorFastOnCmd());
    mainController.b().whileTrue(algaeIntakeSubsystem.reIntakeCmd());
    algaeIntakeSubsystem.setDefaultCommand(algaeIntakeSubsystem.setIntakeMotorSlowOnCmd());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}