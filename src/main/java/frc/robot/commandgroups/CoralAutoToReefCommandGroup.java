// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CoralShooterHoldCmd;
import frc.robot.commands.TagTrackingCmd;
import frc.robot.commands.TagTrackingCmd.AimTarget;
import frc.robot.drivebase.SwerveDrive;
import frc.robot.lib.TagTracking;
import frc.robot.subsystems.CoralShooterSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import java.util.Map;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralAutoToReefCommandGroup extends SequentialCommandGroup {
  SwerveDrive swerveDrive;
  ElevatorSubsystem elevatorSubsystem;
  CoralShooterSubsystem coralShooterSubsystem;
  TagTracking tagTracking = new TagTracking();

  Debouncer tagDebouncer = new Debouncer(1, DebounceType.kFalling);

  public CoralAutoToReefCommandGroup(SwerveDrive swerveDrive, ElevatorSubsystem elevatorSubsystem,
      CoralShooterSubsystem coralShooterSubsystem, int targetFloor, Boolean isLeft, Boolean isAutoTime) {
    this.swerveDrive = swerveDrive;
    this.elevatorSubsystem = elevatorSubsystem;
    this.coralShooterSubsystem = coralShooterSubsystem;

    Command toL3 = Commands.either(
        elevatorSubsystem.toTrdFloorCmd(),
        Commands.none(),
        () -> targetFloor != 2);

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
        coralShooterSubsystem.coralShooterOutCmd().withTimeout(1),
        () -> coralShooterSubsystem.isGetTarget());

    Command putDashboard = Commands.runOnce(() -> {
      SmartDashboard.putBoolean("TagTrackingHasTag", false);
    });

    addCommands(
        Commands.either(
            new SequentialCommandGroup(
                Commands.race(
                    new CoralShooterHoldCmd(coralShooterSubsystem),
                    new SequentialCommandGroup(
                        toL3,
                        new TagTrackingCmd(swerveDrive, isLeft ? AimTarget.LEFT : AimTarget.RIGHT),
                        forwardLittle,
                        elevatorToTargetFloor,
                        new WaitCommand(0.1))),
                autoStopCoralShoot,
                elevatorSubsystem.toDefaultPositionCmd()),
            Commands.none()
                .andThen(putDashboard),
            () -> tagDebouncer.calculate(tagTracking.hasTarget())));
  }
}
