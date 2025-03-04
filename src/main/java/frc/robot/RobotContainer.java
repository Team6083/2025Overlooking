package frc.robot;

import com.fasterxml.jackson.annotation.ObjectIdGenerators.None;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AprilTagAndAutoCmd;
import frc.robot.commands.CoralShooterHoldCmd;
import frc.robot.commands.CoralShooterInWithAutoStopCmd;
import frc.robot.commands.SwerveControlCmd;
import frc.robot.commands.SwerveToTagLeftCmd;
import frc.robot.commands.SwerveToTagRightCmd;
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
  private final CommandGenericHID controlPanel;
  private final SendableChooser<Command> autoChooser;

  private final TagTrackingSubsystem tagTrackingSubsystem;

  private final SequentialCommandGroup takeL2AlgaeCommandGroup;
  private final SequentialCommandGroup takeL3AlgaeCommandGroup;

  public RobotContainer() {
    powerDistribution = new PowerDistribution();
    coralShooterSubsystem = new CoralShooterSubsystem(powerDistribution);
    elevatorSubsystem = new ElevatorSubsystem();
    algaeIntakeSubsystem = new AlgaeIntakeSubsystem(powerDistribution);
    swerveDrive = new SwerveDrive();
    mainController = new CommandXboxController(0);
    controlPanel = new CommandGenericHID(1);
    swerveJoystickCmd = new SwerveControlCmd(swerveDrive, mainController);
    tagTrackingSubsystem = new TagTrackingSubsystem();

    takeL2AlgaeCommandGroup = new SequentialCommandGroup(
        algaeIntakeSubsystem.autoStopRotateCmd(algaeIntakeSubsystem.toAlgaeIntakeDegreeCmd()),
        new ParallelRaceGroup(
            elevatorSubsystem.autoStopCmd(elevatorSubsystem.toGetSecAlgaeCmd()),
            algaeIntakeSubsystem.reIntakeCmd()),
        new ParallelRaceGroup(
            new RunCommand(() -> swerveDrive.drive(-0.4, 0, 0, false), swerveDrive)
                .withTimeout(1.5),
            algaeIntakeSubsystem.reIntakeCmd()));

    takeL3AlgaeCommandGroup = new SequentialCommandGroup(
        algaeIntakeSubsystem.autoStopRotateCmd(algaeIntakeSubsystem.toAlgaeIntakeDegreeCmd()),
        new ParallelRaceGroup(
            elevatorSubsystem.autoStopCmd(elevatorSubsystem.toGetTrdAlgaeCmd()),
            algaeIntakeSubsystem.reIntakeCmd()),
        new ParallelRaceGroup(
            new RunCommand(() -> swerveDrive.drive(-0.4, 0, 0, false), swerveDrive)
                .withTimeout(1.5),
            algaeIntakeSubsystem.reIntakeCmd()));

    NamedCommands.registerCommand("setTuringDegree",
        swerveDrive.setTurningDegreeCmd(0));

    NamedCommands.registerCommand("CoralShooterIn",
        new SequentialCommandGroup(
            new CoralShooterInWithAutoStopCmd(coralShooterSubsystem),
            coralShooterSubsystem.coralShooterSlowOnCmd().withTimeout(0.029)));

    NamedCommands.registerCommand("CoralShooterWithStop",
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
        Commands.either(new SwerveToTagRightCmd(swerveDrive, tagTrackingSubsystem),
            new AprilTagAndAutoCmd(swerveDrive),
            () -> tagTrackingSubsystem.getTv() == 1));

    NamedCommands.registerCommand("AprilTagLeft",
        Commands.either(new SwerveToTagLeftCmd(swerveDrive, tagTrackingSubsystem),
            new AprilTagAndAutoCmd(swerveDrive),
            () -> tagTrackingSubsystem.getTv() == 1));

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
    mainController.back().onTrue(swerveDrive.gyroResetCmd());

    // CoralShooter
    coralShooterSubsystem.setDefaultCommand(
        Commands.either(
            new CoralShooterHoldCmd(coralShooterSubsystem),
            coralShooterSubsystem.coralShooterStopCmd(),
            () -> controlPanel.button(12).getAsBoolean()));
    mainController.rightBumper().whileTrue(coralShooterSubsystem.coralShooterSlowOnCmd());
    mainController.rightBumper().and(mainController.leftBumper())
        .toggleOnTrue(new SequentialCommandGroup(new CoralShooterInWithAutoStopCmd(coralShooterSubsystem),
            coralShooterSubsystem.coralShooterSlowOnCmd().withTimeout(0.029)));

    // Elevator
    mainController.povUp().whileTrue(elevatorSubsystem.toTrdFloorCmd());
    mainController.povLeft().whileTrue(elevatorSubsystem.toSecFloorCmd());
    mainController.povDown().whileTrue(elevatorSubsystem.toDefaultPositionCmd());
    mainController.povRight().whileTrue(elevatorSubsystem.toTopFloorCmd());
    mainController.leftTrigger()
        .whileTrue(Commands.either(
            elevatorSubsystem.moveDownCmd(),
            elevatorSubsystem.manualMoveDownCmd(),
            controlPanel.button(10)));
    mainController.rightTrigger()
        .whileTrue(Commands.either(
            elevatorSubsystem.moveUpCmd(),
            elevatorSubsystem.manualMoveUpCmd(),
            controlPanel.button(10)));
    mainController.start().onTrue(elevatorSubsystem.elevatorReset());

    // ALgaeIntake
    controlPanel.button(1).whileTrue(algaeIntakeSubsystem.manualRotateUpCmd());
    controlPanel.button(3).whileTrue(algaeIntakeSubsystem.manualRotateDownCmd());
    controlPanel.button(7).whileTrue(algaeIntakeSubsystem.toDefaultDegreeCmd());
    controlPanel.button(8).whileTrue(algaeIntakeSubsystem.toAlgaeIntakeDegreeCmd());
    mainController.a().whileTrue(algaeIntakeSubsystem.reIntakeCmd());
    mainController.y().whileTrue(algaeIntakeSubsystem.setIntakeMotorFastOnCmd());
    algaeIntakeSubsystem.setDefaultCommand(algaeIntakeSubsystem.setIntakeMotorSlowOnCmd());

    // Elevator + AlgaeIntake
    controlPanel.button(6).toggleOnTrue(takeL2AlgaeCommandGroup);
    controlPanel.button(5).toggleOnTrue(takeL3AlgaeCommandGroup);

    // TagTracking
    mainController.x().whileTrue(new SwerveToTagRightCmd(swerveDrive, tagTrackingSubsystem));
    mainController.b().whileTrue(new SwerveToTagLeftCmd(swerveDrive, tagTrackingSubsystem));
    // controlPanel.button(2).whileTrue(new SwerveToTagRightCmd(swerveDrive,
    // tagTrackingSubsystem, true));
    // controlPanel.button(4).whileTrue(new SwerveToTagRightCmd(swerveDrive,
    // tagTrackingSubsystem, false));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
