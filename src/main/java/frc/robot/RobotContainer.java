package frc.robot;

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
import frc.robot.commands.SwerveToTagCmd;
import frc.robot.drivebase.SwerveDrive;
import frc.robot.lib.TagTracking;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.CoralShooterSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {
  private final CoralShooterSubsystem coralShooterSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final AlgaeIntakeSubsystem algaeIntakeSubsystem;
  private final SwerveDrive swerveDrive;
  private final CommandXboxController mainController;
  private final CommandGenericHID controlPanel;

  private final SendableChooser<Command> autoChooser;

  private final TagTracking tagTracking;

  private final SequentialCommandGroup takeL2AlgaeCommandGroup;
  private final SequentialCommandGroup takeL3AlgaeCommandGroup;

  public RobotContainer() {
    coralShooterSubsystem = new CoralShooterSubsystem();
    elevatorSubsystem = new ElevatorSubsystem();
    algaeIntakeSubsystem = new AlgaeIntakeSubsystem();
    swerveDrive = new SwerveDrive();
    mainController = new CommandXboxController(0);
    controlPanel = new CommandGenericHID(1);
    tagTracking = new TagTracking();

    takeL2AlgaeCommandGroup = algaeIntakeSubsystem.toAlgaeIntakeDegreeCmd().repeatedly()
        .until(() -> algaeIntakeSubsystem.getAbsoluteError() < 0.5)
        .andThen(new ParallelRaceGroup(
            elevatorSubsystem.toGetSecAlgaeCmd().repeatedly()
                .until(() -> elevatorSubsystem.getAbsoluteError() < 5)),
            algaeIntakeSubsystem.reIntakeCmd())
        .andThen(new ParallelRaceGroup(
            new RunCommand(() -> swerveDrive.drive(-0.4, 0, 0, false), swerveDrive)
                .withTimeout(1.5),
            algaeIntakeSubsystem.reIntakeCmd()));

    takeL3AlgaeCommandGroup = algaeIntakeSubsystem.toAlgaeIntakeDegreeCmd().repeatedly()
        .until(() -> algaeIntakeSubsystem.getAbsoluteError() < 0.5)
        .andThen(new ParallelRaceGroup(
            elevatorSubsystem.toGetTrdAlgaeCmd().repeatedly()
                .until(() -> elevatorSubsystem.getAbsoluteError() < 5),
            algaeIntakeSubsystem.reIntakeCmd()))
        .andThen(new ParallelRaceGroup(
            new RunCommand(() -> swerveDrive.drive(-0.4, 0, 0, false), swerveDrive)
                .withTimeout(1.5),
            algaeIntakeSubsystem.reIntakeCmd()));
    
    namedCommands();

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    SmartDashboard.putData("AutoChooser", autoChooser);
    SmartDashboard.putData("CoralShooterSubsystem", coralShooterSubsystem);
    SmartDashboard.putData("ElevatorSubsystem", elevatorSubsystem);
    SmartDashboard.putData("AlgaeIntakeSubsystem", algaeIntakeSubsystem);
    SmartDashboard.putData("SwerveDrive", swerveDrive);

    configureBindings();
  }

  private void namedCommands() {
    NamedCommands.registerCommand("SetTurningDegree",
        swerveDrive.setTurningDegreeCmd(0).withTimeout(0.1));

    NamedCommands.registerCommand("CoralShooterIn",
        new CoralShooterInWithAutoStopCmd(coralShooterSubsystem)
            .andThen(coralShooterSubsystem.coralShooterOnCmd().withTimeout(0.029)));

    NamedCommands.registerCommand("CoralShooterWithStop",
        coralShooterSubsystem.coralShooterOnCmd().withTimeout(1)
            .andThen(coralShooterSubsystem.coralShooterStopCmd()));

    NamedCommands.registerCommand("ErToSec",
        elevatorSubsystem.toSecFloorCmd());

    NamedCommands.registerCommand("ErToTrd",
        elevatorSubsystem.toTrdFloorCmd());

    NamedCommands.registerCommand("ErToFour",
        elevatorSubsystem.toTopFloorCmd().repeatedly()
            .until(() -> elevatorSubsystem.getAbsoluteError() < 5));

    NamedCommands.registerCommand("ErDown",
        elevatorSubsystem.toDefaultPositionCmd());

    NamedCommands.registerCommand("AprilTagRight",
        Commands.either(new SwerveToTagCmd(swerveDrive, false),
            new AprilTagAndAutoCmd(swerveDrive),
            () -> tagTracking.getTv() == 1));

    NamedCommands.registerCommand("AprilTagLeft",
        Commands.either(new SwerveToTagCmd(swerveDrive, true),
            new AprilTagAndAutoCmd(swerveDrive),
            () -> tagTracking.getTv() == 1));

    NamedCommands.registerCommand("AlgaeIntake",
        algaeIntakeSubsystem.setIntakeMotorOnCmd());
  }

  private void configureBindings() {
    // SwerveDrive
    swerveDrive.setDefaultCommand(new SwerveControlCmd(swerveDrive, mainController));
    // used LeftBumper to switch between fast and slow mode
    mainController.back().onTrue(swerveDrive.gyroResetCmd());

    // CoralShooter
    controlPanel.button(11).whileTrue(
        new CoralShooterHoldCmd(coralShooterSubsystem));
    mainController.rightBumper().whileTrue(coralShooterSubsystem.coralShooterOnCmd());
    mainController.rightBumper().and(mainController.leftBumper())
        .toggleOnTrue(new SequentialCommandGroup(new CoralShooterInWithAutoStopCmd(coralShooterSubsystem),
            coralShooterSubsystem.coralShooterOnCmd().withTimeout(0.029)));

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
    mainController.y().whileTrue(algaeIntakeSubsystem.setIntakeMotorOnCmd());
    controlPanel.button(12).onTrue(algaeIntakeSubsystem.EnablePID());
    controlPanel.button(12).onFalse(algaeIntakeSubsystem.DisablePID());

    // Elevator + AlgaeIntake
    controlPanel.button(6).toggleOnTrue(takeL2AlgaeCommandGroup);
    controlPanel.button(5).toggleOnTrue(takeL3AlgaeCommandGroup);

    // TagTracking
    mainController.x().whileTrue(new SwerveToTagCmd(swerveDrive, false));
    mainController.b().whileTrue(new SwerveToTagCmd(swerveDrive, true));
    controlPanel.button(2).whileTrue(new SwerveToTagCmd(swerveDrive, true));
    controlPanel.button(4).whileTrue(new SwerveToTagCmd(swerveDrive, false));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
