package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoCoralAndElevatorCmd;
import frc.robot.commands.CoralShooterHoldCmd;
import frc.robot.commands.CoralShooterInWithAutoStopCmd;
import frc.robot.commands.SwerveControlCmd
import frc.robot.commands.TakeAlgaeCommandGroup;
import frc.robot.drivebase.SwerveDrive;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.CoralShooterSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import java.util.Map;
import java.util.function.Supplier;

public class RobotContainer {
  private final CoralShooterSubsystem coralShooterSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final AlgaeIntakeSubsystem algaeIntakeSubsystem;
  private final SwerveDrive swerveDrive;

  private final CommandXboxController mainController = new CommandXboxController(0);
  private final CommandGenericHID controlPanel = new CommandGenericHID(1);

  private final Supplier<Boolean> elevatorBypassSafety = () -> controlPanel.button(9).getAsBoolean();

  private final SendableChooser<Command> autoChooser;

    private Supplier<Integer> targetFloor;

  public RobotContainer() {
    Supplier<Boolean> elevatorUsePID = () -> controlPanel.button(10).getAsBoolean();
    Supplier<Boolean> algaeRotateUsePID = () -> controlPanel.button(12).getAsBoolean();

    coralShooterSubsystem = new CoralShooterSubsystem();
    elevatorSubsystem = new ElevatorSubsystem(elevatorUsePID, elevatorBypassSafety);
    algaeIntakeSubsystem = new AlgaeIntakeSubsystem(algaeRotateUsePID);
    swerveDrive = new SwerveDrive();

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
            .andThen(coralShooterSubsystem.coralShooterInCmd().withTimeout(0.029)));

    NamedCommands.registerCommand("CoralShooterWithStop",
        coralShooterSubsystem.coralShooterOutCmd().withTimeout(1));

    NamedCommands.registerCommand("ErToSec",
        elevatorSubsystem.toSecFloorCmd());

    NamedCommands.registerCommand("ErToTrd",
        elevatorSubsystem.toTrdFloorCmd());

    NamedCommands.registerCommand("ErToFour",
        elevatorSubsystem.toTopFloorCmd().repeatedly()
            .until(() -> elevatorSubsystem.isAtTargetHeight()));

    NamedCommands.registerCommand("ErDown",
        elevatorSubsystem.toDefaultPositionCmd());

    // NamedCommands.registerCommand("AprilTagRight",
    // Commands.either(new SwerveToTagCmd(swerveDrive, false).withTimeout(4),
    // swerveDrive.driveForwardCmd().withTimeout(2),
    // () -> tagTracking.getTv() == 1));

    // NamedCommands.registerCommand("AprilTagLeft",
    // Commands.either(new SwerveToTagCmd(swerveDrive, true).withTimeout(4),
    // swerveDrive.driveForwardCmd().withTimeout(2),
    // () -> tagTracking.getTv() == 1));

    NamedCommands.registerCommand("AlgaeIntake",
        algaeIntakeSubsystem.intakeCmd());

    NamedCommands.registerCommand("CoralShooterHold",
        new CoralShooterHoldCmd(coralShooterSubsystem));
  }

  private void configureBindings() {
    // SwerveDrive
    swerveDrive.setDefaultCommand(new SwerveControlCmd(
        swerveDrive, mainController, elevatorSubsystem, elevatorBypassSafety));
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
    mainController.rightBumper().whileTrue(coralShooterSubsystem.coralShooterOutCmd());
    mainController.rightBumper().and(mainController.leftBumper())
        .toggleOnTrue(new SequentialCommandGroup(new CoralShooterInWithAutoStopCmd(coralShooterSubsystem),
            coralShooterSubsystem.coralShooterInCmd().withTimeout(0.029)));

    mainController.button(10).whileTrue(coralShooterSubsystem.coralShooterReverseShootCmd());

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
    mainController.y().whileTrue(algaeIntakeSubsystem.manualRotateUpCmd());
    mainController.a().whileTrue(algaeIntakeSubsystem.manualRotateDownCmd());
    controlPanel.button(7).whileTrue(algaeIntakeSubsystem.toDefaultDegreeCmd());
    controlPanel.button(8).whileTrue(
        new SequentialCommandGroup(
            algaeIntakeSubsystem.toAlgaeIntakeDegreeCmd(),
            algaeIntakeSubsystem.intakeCmd()));
    controlPanel.button(3).whileTrue(algaeIntakeSubsystem.reverseIntakeCmd());
    mainController.x().whileTrue(algaeIntakeSubsystem.intakeCmd());
    mainController.b().whileTrue(algaeIntakeSubsystem.reverseIntakeCmd());

    // Elevator + AlgaeIntake
    controlPanel.button(6).whileTrue(
        new TakeAlgaeCommandGroup(swerveDrive, elevatorSubsystem, algaeIntakeSubsystem, 2));
    controlPanel.button(5).whileTrue(
        new TakeAlgaeCommandGroup(swerveDrive, elevatorSubsystem, algaeIntakeSubsystem, 3));

    // switch floor
    controlPanel.button(1).onTrue(setTargetFloor(2));
    controlPanel.button(3).onTrue(setTargetFloor(3));
    controlPanel.button(5).onTrue(setTargetFloor(4));

    Map<Integer, Command> coralLeftMap = Map.of(
        2, new AutoCoralAndElevatorCmd(
            swerveDrive, elevatorSubsystem, coralShooterSubsystem, 2, true),
        3, new AutoCoralAndElevatorCmd(
            swerveDrive, elevatorSubsystem, coralShooterSubsystem, 3, true),
        4, new AutoCoralAndElevatorCmd(
            swerveDrive, elevatorSubsystem, coralShooterSubsystem, 4, true));

    controlPanel.button(2).whileTrue(Commands.select(coralLeftMap, targetFloor));

    Map<Integer, Command> coralRightMap = Map.of(
        2, new AutoCoralAndElevatorCmd(
            swerveDrive, elevatorSubsystem, coralShooterSubsystem, 2, false),
        3, new AutoCoralAndElevatorCmd(
            swerveDrive, elevatorSubsystem, coralShooterSubsystem, 3, false),
        4, new AutoCoralAndElevatorCmd(
            swerveDrive, elevatorSubsystem, coralShooterSubsystem, 4, false));

    controlPanel.button(4).whileTrue(Commands.select(coralRightMap, targetFloor));
  }

  private Command setTargetFloor(int floor) {
    return Commands.runOnce(() -> this.targetFloor = () -> floor);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
