// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.drivebase.SwerveDrive;
import frc.robot.subsystems.CoralShooterSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCoralAndElevatorCmd extends SequentialCommandGroup {
  /** Creates a new CoralRightCmd. */
  SwerveDrive swerveDrive;
  ElevatorSubsystem elevatorSubsystem;
  CoralShooterSubsystem coralShooterSubsystem;

  public AutoCoralAndElevatorCmd(SwerveDrive swerveDrive, ElevatorSubsystem elevatorSubsystem,
      CoralShooterSubsystem coralShooterSubsystem, int targetFloor, Boolean isLeft) {
    this.swerveDrive = swerveDrive;
    this.elevatorSubsystem = elevatorSubsystem;

    Command swerveToTag = Commands.either(
        new SwerveToReefCmd(swerveDrive, isLeft),
        new SwerveToReefCmd(swerveDrive, isLeft),
        () -> isLeft);

    Command forwardLittle = swerveDrive
        .runEnd(
            () -> swerveDrive.drive(0.45, 0, 0, false),
            () -> swerveDrive.drive(0, 0, 0, false))
        .repeatedly()
        .withTimeout(0.35);

    Map<Integer, Command> elevatorMoveToHeightMap = Map
        .of(
            2, elevatorSubsystem.toSecFloorCmd(),
            3, elevatorSubsystem.toTrdFloorCmd(),
            4, elevatorSubsystem.toTopFloorCmd());

    Command elevatorToTargetFloor = Commands
        .select(
            elevatorMoveToHeightMap,
            () -> targetFloor)
        .repeatedly()
        .until(() -> elevatorSubsystem.isAtTargetHeight());

    Command autoStopCoralShoot = coralShooterSubsystem
        .coralShooterOutCmd()
        .until(() -> !coralShooterSubsystem.isGetTarget());

    Command elevatorToDefaultPosition = elevatorSubsystem
        .toDefaultPositionCmd().repeatedly()
        .until(() -> elevatorSubsystem.isAtTargetHeight());

    Command backwardLittle = swerveDrive
        .runEnd(
            () -> swerveDrive.drive(-0.75, 0, 0, false),
            () -> swerveDrive.drive(0, 0, 0, false))
        .repeatedly()
        .withTimeout(0.5);

    addCommands(
        Commands.race(
            new CoralShooterHoldCmd(coralShooterSubsystem),
            new SequentialCommandGroup(
                swerveToTag,
                forwardLittle,
                elevatorToTargetFloor)),
        autoStopCoralShoot,
        elevatorToDefaultPosition,
        backwardLittle);
  }
}
