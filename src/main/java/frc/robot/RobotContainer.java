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
import frc.robot.commands.CoralShooterHoldCmd;
import frc.robot.commands.SwerveControlCmd;
import frc.robot.commands.SwerveTagTrackingCmd;
import frc.robot.commands.SwerveToReef;
import frc.robot.drivebase.SwerveDrive;
import frc.robot.lib.PowerDistribution;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.CoralShooterSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TagTrackingSubsystem;

public class RobotContainer {
  private final PowerDistribution powerDistribution;
  private final CoralShooterSubsystem coralShooterSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final AlgaeIntakeSubsystem algaeIntakeSubsystem;
  private final SwerveDrive swerveDrive;
  private final SwerveControlCmd swerveJoystickCmd;
  private final CommandXboxController mainController;
  private final SendableChooser<Command> autoChooser;
  private final TagTrackingSubsystem tagTrackingSubsystem;
  private final SwerveTagTrackingCmd swerveTagTrackingCmd;
  private final SwerveToReef swerveToReefLeftCmd;
  private final SwerveToReef swerveToReefRightCmd;
  public RobotContainer() {
    powerDistribution = new PowerDistribution();
    coralShooterSubsystem = new CoralShooterSubsystem(powerDistribution);
    elevatorSubsystem = new ElevatorSubsystem();
    algaeIntakeSubsystem = new AlgaeIntakeSubsystem(powerDistribution);
    swerveDrive = new SwerveDrive();
    mainController = new CommandXboxController(0);
    swerveJoystickCmd = new SwerveControlCmd(swerveDrive, mainController);
    tagTrackingSubsystem = new TagTrackingSubsystem();
    swerveTagTrackingCmd = new SwerveTagTrackingCmd(swerveDrive, tagTrackingSubsystem);
    swerveToReefLeftCmd = new SwerveToReef(swerveDrive, tagTrackingSubsystem, "left");
    swerveToReefRightCmd = new SwerveToReef(swerveDrive, tagTrackingSubsystem, "right");

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    SmartDashboard.putData("AutoChooser", autoChooser);
    SmartDashboard.putData("CoralShooterSubsystem", coralShooterSubsystem);
    SmartDashboard.putData("ElevatorSubsystem", elevatorSubsystem);
    SmartDashboard.putData("AlgaeIntakeSubsystem", algaeIntakeSubsystem);
    SmartDashboard.putData("SwerveDrive", swerveDrive);
    SmartDashboard.putData("TagTracking", tagTrackingSubsystem);
    configureBindings();
  }

  private void configureBindings() {
    // SwerveDrive
    coralShooterSubsystem.setDefaultCommand(new CoralShooterHoldCmd(coralShooterSubsystem));
    swerveDrive.setDefaultCommand(swerveJoystickCmd);
    mainController.back().onTrue(swerveDrive.gyroResetCmd());
    mainController.y().whileTrue(new SequentialCommandGroup(
        swerveTagTrackingCmd,
        swerveToReefRightCmd
    ));

    // CoralShooter
    mainController.rightBumper().whileTrue(coralShooterSubsystem.coralShooterSlowOnCmd());
    // Elevator
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

    // ALgaeIntake
    mainController.y().whileTrue(algaeIntakeSubsystem.rotateUpCmd());
    mainController.a().whileTrue(algaeIntakeSubsystem.rotateDownCmd());
    mainController.x().whileTrue(algaeIntakeSubsystem.setIntakeMotorFastOnCmd());
    mainController.b().whileTrue(algaeIntakeSubsystem.reIntakeCmd());
    algaeIntakeSubsystem.setDefaultCommand(algaeIntakeSubsystem.setIntakeMotorSlowOnCmd());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}