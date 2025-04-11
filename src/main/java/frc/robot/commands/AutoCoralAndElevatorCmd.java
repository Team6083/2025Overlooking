// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TagTrackingCmd.AimTarget;
import frc.robot.drivebase.SwerveDrive;
import frc.robot.subsystems.CoralShooterSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import java.util.Map;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCoralAndElevatorCmd extends SequentialCommandGroup {
  SwerveDrive swerveDrive;
  ElevatorSubsystem elevatorSubsystem;
  CoralShooterSubsystem coralShooterSubsystem;

  public AutoCoralAndElevatorCmd(SwerveDrive swerveDrive, ElevatorSubsystem elevatorSubsystem,
      CoralShooterSubsystem coralShooterSubsystem, int targetFloor, Boolean isLeft, Boolean isAutoTime) {
    this.swerveDrive = swerveDrive;
    this.elevatorSubsystem = elevatorSubsystem;
    this.coralShooterSubsystem = coralShooterSubsystem;

    Command forwardLittle = swerveDrive
        .runEnd(
            () -> swerveDrive.drive(0.45, 0, 0, false),
            () -> swerveDrive.drive(0, 0, 0, false))
        .repeatedly()
        .withTimeout(0.52);

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

    Command autoStopCoralShoot = Commands.either(
        coralShooterSubsystem
            .coralShooterOutCmd()
            .until(() -> !coralShooterSubsystem.isGetTarget()),
        coralShooterSubsystem.coralShooterOutCmd().withTimeout(1.5),
        () -> coralShooterSubsystem.isGetTarget());

    addCommands(
        Commands.race(
            new CoralShooterHoldCmd(coralShooterSubsystem),
            new SequentialCommandGroup(
                new TagTrackingCmd(swerveDrive, isLeft ? AimTarget.LEFT : AimTarget.RIGHT),
                forwardLittle,
                elevatorToTargetFloor)),
        autoStopCoralShoot,
        elevatorSubsystem.toDefaultPositionCmd());
  }
}
