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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CoralShooterHoldCmd;
import frc.robot.commands.CoralShooterInWithAutoStopCmd;
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

    private final SequentialCommandGroup takeL2AlgaeCommandGroup;
    private final SequentialCommandGroup takeL3AlgaeCommandGroup;

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
        takeL2AlgaeCommandGroup = new SequentialCommandGroup(
                algaeIntakeSubsystem.autoStopRotateCmd(algaeIntakeSubsystem.toAlgaeIntakeDegreeCmd()),
                new ParallelRaceGroup(elevatorSubsystem.autoStopCmd(elevatorSubsystem.toGetSecAlgaeCmd()),
                        algaeIntakeSubsystem.reIntakeCmd()),
                new ParallelRaceGroup(new RunCommand(() -> swerveDrive.drive(-0.4, 0, 0, false), swerveDrive)
                        .withTimeout(1.5),
                        algaeIntakeSubsystem.reIntakeCmd()));

        takeL3AlgaeCommandGroup = new SequentialCommandGroup(
                algaeIntakeSubsystem.autoStopRotateCmd(algaeIntakeSubsystem.toAlgaeIntakeDegreeCmd()),
                new ParallelRaceGroup(elevatorSubsystem.autoStopCmd(elevatorSubsystem.toGetTrdAlgaeCmd()),
                        algaeIntakeSubsystem.reIntakeCmd()),
                new ParallelRaceGroup(new RunCommand(() -> swerveDrive.drive(-0.4, 0, 0, false), swerveDrive)
                        .withTimeout(1.5),
                        algaeIntakeSubsystem.reIntakeCmd()));

        NamedCommands.registerCommand("CoralShooterIn",
                new SequentialCommandGroup(new CoralShooterInWithAutoStopCmd(coralShooterSubsystem),
                        coralShooterSubsystem.coralShooterSlowOnCmd().withTimeout(0.029)));

        NamedCommands.registerCommand("CoralShooterWithAutoStop",
                coralShooterSubsystem.coralShooterSlowOnCmd().withTimeout(1)
                        .andThen(coralShooterSubsystem.coralShooterStopCmd()));

        NamedCommands.registerCommand("ErToSec",
                elevatorSubsystem.toSecFloorCmd());

        NamedCommands.registerCommand("ErToTrd",
                elevatorSubsystem.toTrdFloorCmd());

        NamedCommands.registerCommand("ErToFour",
                elevatorSubsystem.toTopFloorCmd());

        NamedCommands.registerCommand("ErDown",
                elevatorSubsystem.toDefaultPositionCmd());

        NamedCommands.registerCommand("AprilTagRight",
                new SequentialCommandGroup(
                        new SwerveTagTrackingCmd(swerveDrive, tagTrackingSubsystem),
                        new SwerveToReef(swerveDrive, tagTrackingSubsystem, "right")));

        NamedCommands.registerCommand("AprilTagLeft",
                new SequentialCommandGroup(
                        new SwerveTagTrackingCmd(swerveDrive, tagTrackingSubsystem),
                        new SwerveToReef(swerveDrive, tagTrackingSubsystem, "left")));

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
        swerveDrive.setDefaultCommand(swerveJoystickCmd);
        mainController.back().whileTrue(swerveDrive.gyroResetCmd());

        // mainController.y().whileTrue(swerveDrive.setTurningDegreeCmd(0));
        // mainController.y().whileTrue(new SequentialCommandGroup(
        // swerveTagTrackingCmd,
        // swerveToReefRightCmd
        // ));
        // mainController.y().whileTrue(swerveToReefLeftCmd);
        // mainController.a().whileTrue(new SequentialCommandGroup(
        // new SwerveTagTrackingCmd(swerveDrive, tagTrackingSubsystem),
        // swerveToReefLeftCmd
        // ));

        // mainController.y().whileTrue(swerveDrive.runEnd(
        // () -> swerveDrive.drive(0.1, 0, 0, false),
        // () -> swerveDrive.stop()));

        // CoralShooter
        coralShooterSubsystem.setDefaultCommand(new CoralShooterHoldCmd(coralShooterSubsystem));
        mainController.rightBumper().whileTrue(coralShooterSubsystem.coralShooterSlowOnCmd());
        mainController.rightBumper().and(mainController.pov(90))
                .whileTrue(new SequentialCommandGroup(new CoralShooterInWithAutoStopCmd(coralShooterSubsystem),
                        coralShooterSubsystem.coralShooterSlowOnCmd().withTimeout(0.029)));
        // Elevator
        mainController.povUp().whileTrue(elevatorSubsystem.toTrdFloorCmd());
        mainController.povDown().whileTrue(elevatorSubsystem.toDefaultPositionCmd());
        mainController.povLeft().whileTrue(elevatorSubsystem.toSecFloorCmd());
        mainController.leftTrigger()
                .whileTrue(Commands.either(
                        elevatorSubsystem.manualMoveDownCmd(),
                        elevatorSubsystem.moveDownCmd(),
                        mainController.povRight()));
        mainController.rightTrigger()
                .whileTrue(Commands.either(
                        elevatorSubsystem.manualMoveUpCmd(),
                        elevatorSubsystem.moveUpCmd(),
                        mainController.povRight()));
        mainController.start().onTrue(elevatorSubsystem.elevatorReset());

        // ALgaeIntake
        mainController.b().whileTrue(takeL2AlgaeCommandGroup);
        mainController.x().whileTrue(takeL3AlgaeCommandGroup);
        mainController.y().whileTrue(algaeIntakeSubsystem.toDefaultDegreeCmd());
        mainController.a().whileTrue(algaeIntakeSubsystem.toAlgaeIntakeDegreeCmd());
        // mainController.b().whileTrue(algaeIntakeSubsystem.reIntakeCmd());
        // mainController.x().whileTrue(algaeIntakeSubsystem.setIntakeMotorFastOnCmd());
        algaeIntakeSubsystem.setDefaultCommand(algaeIntakeSubsystem.setIntakeMotorSlowOnCmd());
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}