package frc.robot;

import java.util.function.Supplier;

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
    mainController = new CommandXboxController(0);
    controlPanel = new CommandGenericHID(1);

    Supplier<Boolean> elevatorUsePID = () -> controlPanel.button(10).getAsBoolean();
    Supplier<Boolean> elevatorBypassLimitSW = () -> controlPanel.button(9).getAsBoolean();
    Supplier<Boolean> algaeRotateUsePID = () -> controlPanel.button(12).getAsBoolean();

    coralShooterSubsystem = new CoralShooterSubsystem();
    elevatorSubsystem = new ElevatorSubsystem(elevatorUsePID, elevatorBypassLimitSW);
    algaeIntakeSubsystem = new AlgaeIntakeSubsystem(algaeRotateUsePID);
    swerveDrive = new SwerveDrive();

    tagTracking = new TagTracking();

    takeL2AlgaeCommandGroup = new ParallelRaceGroup(
        elevatorSubsystem.toGetSecAlgaeCmd().repeatedly()
            .until(() -> elevatorSubsystem.getAbsoluteError() < 5),
        algaeIntakeSubsystem.reverseIntakeCmd())
        .andThen(new ParallelRaceGroup(
            new RunCommand(() -> swerveDrive.drive(-0.4, 0, 0, false), swerveDrive)
                .withTimeout(1.5),
            algaeIntakeSubsystem.reverseIntakeCmd()));

    takeL3AlgaeCommandGroup = new ParallelRaceGroup(
        elevatorSubsystem.toGetTrdAlgaeCmd().repeatedly()
            .until(() -> elevatorSubsystem.getAbsoluteError() < 5),
        algaeIntakeSubsystem.reverseIntakeCmd())
        .andThen(new ParallelRaceGroup(
            new RunCommand(() -> swerveDrive.drive(-0.4, 0, 0, false), swerveDrive)
                .withTimeout(1.5),
            algaeIntakeSubsystem.reverseIntakeCmd()));

    registerNamedCommands();

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    SmartDashboard.putData("AutoChooser", autoChooser);

    SmartDashboard.putData("CoralShooterSubsystem", coralShooterSubsystem);
    SmartDashboard.putData("ElevatorSubsystem", elevatorSubsystem);
    SmartDashboard.putData("AlgaeIntakeSubsystem", algaeIntakeSubsystem);
    SmartDashboard.putData("SwerveDrive", swerveDrive);

    configureBindings();
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("SetTurningDegree",
        swerveDrive.setTurningDegreeCmd(0).withTimeout(0.1));

    NamedCommands.registerCommand("CoralShooterIn",
        new CoralShooterInWithAutoStopCmd(coralShooterSubsystem)
            .andThen(coralShooterSubsystem.coralShooterOnCmd().withTimeout(0.029)));

    NamedCommands.registerCommand("CoralShooterWithStop",
        coralShooterSubsystem.coralShooterOnCmd().withTimeout(1));

    NamedCommands.registerCommand("ErToSec",
        elevatorSubsystem.toSecFloorCmd());

    NamedCommands.registerCommand("ErToTrd",
        elevatorSubsystem.toTrdFloorCmd());

    NamedCommands.registerCommand("ErToFour",
        elevatorSubsystem.toTopFloorCmd().repeatedly()
            .until(() -> elevatorSubsystem.isAtTargetHeight()));

    NamedCommands.registerCommand("ErDown",
        elevatorSubsystem.toDefaultPositionCmd());

    NamedCommands.registerCommand("AprilTagRight",
        Commands.either(new SwerveToTagCmd(swerveDrive, false),
            swerveDrive.driveForwardCmd().withTimeout(2),
            () -> tagTracking.getTv() == 1));

    NamedCommands.registerCommand("AprilTagLeft",
        Commands.either(new SwerveToTagCmd(swerveDrive, true),
            swerveDrive.driveForwardCmd().withTimeout(2),
            () -> tagTracking.getTv() == 1));

    NamedCommands.registerCommand("AlgaeIntake",
        algaeIntakeSubsystem.intakeCmd());
  }

  private void configureBindings() {
    // SwerveDrive
    swerveDrive.setDefaultCommand(new SwerveControlCmd(swerveDrive, mainController));
    // used LeftBumper to switch between fast and slow mode
    mainController.back().onTrue(swerveDrive.gyroResetCmd());

    // CoralShooter
    var coralShooterDefaultCmd = Commands.either(
        new CoralShooterHoldCmd(coralShooterSubsystem),
        Commands.none(),
        controlPanel.button(11));
    controlPanel.button(11).onChange(coralShooterSubsystem.runOnce(() -> {
    }));

    coralShooterSubsystem.setDefaultCommand(coralShooterDefaultCmd);
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
    mainController.a().whileTrue(algaeIntakeSubsystem.reverseIntakeCmd());
    mainController.y().whileTrue(algaeIntakeSubsystem.intakeCmd());

    // Elevator + AlgaeIntake
    controlPanel.button(6).whileTrue(takeL2AlgaeCommandGroup);
    controlPanel.button(5).whileTrue(takeL3AlgaeCommandGroup);

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
