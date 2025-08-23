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
import frc.robot.commands.TagTrackingCmd;
import frc.robot.commands.TagTrackingCmd.AimTarget;
import frc.robot.drivebase.SwerveDrive;
import frc.robot.lib.TagTracking;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TakeAlgaeCommandGroup extends SequentialCommandGroup {
  /** Creates a new TakeL2AlgaeCommandGroup. */
  SwerveDrive swerveDrive;
  ElevatorSubsystem elevatorSubsystem;
  AlgaeIntakeSubsystem algaeIntakeSubsystem;
  TagTracking tagTracking = new TagTracking();

  Debouncer tagDebouncer = new Debouncer(1, DebounceType.kFalling);

  public TakeAlgaeCommandGroup(SwerveDrive swerveDrive,
      ElevatorSubsystem elevatorSubsystem, AlgaeIntakeSubsystem algaeIntakeSubsystem) {
    this.swerveDrive = swerveDrive;
    this.elevatorSubsystem = elevatorSubsystem;
    this.algaeIntakeSubsystem = algaeIntakeSubsystem;

    Command elevatorToTargetHeight = Commands.either(
        elevatorSubsystem.toGetSecAlgaeCmd(),
        elevatorSubsystem.toGetTrdAlgaeCmd(),
        () -> {
          switch ((int) tagTracking.getBestTargetId()) {
            case 6, 8, 10, 17, 19, 21:
              return true;
            case 7, 9, 11, 18, 20, 22:
              return false;
            default:
              return true;
          }
        })
        .andThen(Commands.waitUntil(() -> elevatorSubsystem.isAtTargetHeight()));

    Command forwardLittle = swerveDrive
        .runEnd(
            () -> swerveDrive.drive(0.75, 0, 0, false),
            () -> swerveDrive.drive(0, 0, 0, false))
        .repeatedly()
        .withTimeout(0.57);

    Command algaeToTargetAngle = new SequentialCommandGroup(
        algaeIntakeSubsystem.toAlgaeIntakeDegreeCmd(),
        algaeIntakeSubsystem.intakeCmd().withTimeout(0.8));

    Command backwardLittle = swerveDrive
        .runEnd(
            () -> swerveDrive.drive(-0.4, 0, 0, false),
            () -> swerveDrive.drive(0, 0, 0, false))
        .repeatedly()
        .withTimeout(1.3);

    Command putDashboard = Commands.runOnce(() -> {
      SmartDashboard.putBoolean("TagTrackingHasTag", false);
    });

    addCommands(
        Commands.either(
            new SequentialCommandGroup(
                elevatorToTargetHeight,
                new TagTrackingCmd(swerveDrive, AimTarget.CENTER),
                forwardLittle,
                algaeToTargetAngle,
                Commands.race(
                    algaeIntakeSubsystem.intakeCmd(),
                    backwardLittle)),
            Commands.none()
                .andThen(putDashboard),
            () -> tagDebouncer.calculate(tagTracking.hasTarget())));
  }
}