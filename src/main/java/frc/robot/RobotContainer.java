package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.Rev2mDistanceSensor.Port;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commandgroups.CoralAutoToReefCommandGroup;
import frc.robot.commandgroups.TakeAlgaeCommandGroup;
import frc.robot.commands.CoralShooterHoldCmd;
import frc.robot.commands.SwerveControlCmd;
import frc.robot.drivebase.SwerveDrive;
import frc.robot.lib.Elastic;
import frc.robot.lib.Elastic.Notification.NotificationLevel;
import frc.robot.lib.sensor.distance.Rev2mDistanceSensor;
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
  private final Rev2mDistanceSensor distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);

  private final CommandXboxController mainController = new CommandXboxController(0);
  private final CommandGenericHID controlPanel = new CommandGenericHID(1);

  private final Supplier<Boolean> elevatorBypassSafety = () -> controlPanel.button(9).getAsBoolean();

  private final SendableChooser<Command> autoChooser;

  private Supplier<Integer> targetFloor = () -> 2;

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

    SmartDashboard.putNumber("TargetFloor", 2);

    configureBindings();
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("CoralIn",
        coralShooterSubsystem.coralShooterAutoInCmd()
            .andThen(coralShooterSubsystem.coralShooterInCmd().withTimeout(0.035)));

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

    NamedCommands.registerCommand("AlgaeIntakeToDefaultPosition",
        algaeIntakeSubsystem.toDefaultDegreeCmd());

    NamedCommands.registerCommand("CoralShooterHold",
        new CoralShooterHoldCmd(coralShooterSubsystem));

    NamedCommands.registerCommand("CoralLeftL2",
        new CoralAutoToReefCommandGroup(
            swerveDrive, elevatorSubsystem, coralShooterSubsystem, 2, true, true));

    NamedCommands.registerCommand("CoralRightL2",
        new CoralAutoToReefCommandGroup(
            swerveDrive, elevatorSubsystem, coralShooterSubsystem, 2, false, true));

    NamedCommands.registerCommand("CoralLeftL4",
        new CoralAutoToReefCommandGroup(
            swerveDrive, elevatorSubsystem, coralShooterSubsystem, 4, true, true));

    NamedCommands.registerCommand("CoralRightL4",
        new CoralAutoToReefCommandGroup(
            swerveDrive, elevatorSubsystem, coralShooterSubsystem, 4, false, true));

    NamedCommands.registerCommand("TakeAlgae",
        new TakeAlgaeCommandGroup(
            swerveDrive, elevatorSubsystem, algaeIntakeSubsystem));

  }

  private void configureBindings() {
    // SwerveDrive
    swerveDrive.setDefaultCommand(new SwerveControlCmd(
        swerveDrive, mainController));
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
        .toggleOnTrue(new SequentialCommandGroup(coralShooterSubsystem.coralShooterAutoInCmd(),
            coralShooterSubsystem.coralShooterInCmd()
                .withTimeout(ConfigChooser.CoralShooter.getDouble("kCoralInTimeOut"))));
    controlPanel.button(6)
        .toggleOnTrue(new SequentialCommandGroup(coralShooterSubsystem.coralShooterAutoInCmd(),
            coralShooterSubsystem.coralShooterInCmd()
                .withTimeout(ConfigChooser.CoralShooter.getDouble("kCoralInTimeOut"))));
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
    mainController.x().whileTrue(new SequentialCommandGroup(
        algaeIntakeSubsystem.toAlgaeIntakeDegreeCmd(),
        algaeIntakeSubsystem.intakeCmd()));
    mainController.b().whileTrue(algaeIntakeSubsystem.reverseIntakeCmd());
    controlPanel.button(8).whileTrue(algaeIntakeSubsystem.toDefaultDegreeCmd());

    // switch floor
    controlPanel.button(3)
        .onTrue(setTargetFloor(2)
            .andThen(Commands.runOnce(
                () -> Elastic.sendNotification("Floor Changed", "Floor 2 selected"))));
    controlPanel.button(2)
        .onTrue(setTargetFloor(3)
            .andThen(Commands.runOnce(
                () -> Elastic.sendNotification("Floor Changed", "Floor 3 selected"))));
    controlPanel.button(1)
        .onTrue(setTargetFloor(4)
            .andThen(Commands.runOnce(
                () -> Elastic.sendNotification("Floor Changed", "Floor 4 selected"))));

    Map<Integer, Command> oneButtonAlgaeMap = Map.of(
        2, new TakeAlgaeCommandGroup(
            swerveDrive, elevatorSubsystem, algaeIntakeSubsystem),
        3, new TakeAlgaeCommandGroup(
            swerveDrive, elevatorSubsystem, algaeIntakeSubsystem));

    controlPanel.button(7).whileTrue(Commands.select(oneButtonAlgaeMap, () -> targetFloor.get()));

    Map<Integer, Command> coralLeftMap = Map.of(
        2, new CoralAutoToReefCommandGroup(
            swerveDrive, elevatorSubsystem, coralShooterSubsystem, 2, true, false),
        3, new CoralAutoToReefCommandGroup(
            swerveDrive, elevatorSubsystem, coralShooterSubsystem, 3, true, false),
        4, new CoralAutoToReefCommandGroup(
            swerveDrive, elevatorSubsystem, coralShooterSubsystem, 4, true, false));

    Map<Integer, Command> coralRightMap = Map.of(
        2, new CoralAutoToReefCommandGroup(
            swerveDrive, elevatorSubsystem, coralShooterSubsystem, 2, false, false),
        3, new CoralAutoToReefCommandGroup(
            swerveDrive, elevatorSubsystem, coralShooterSubsystem, 3, false, false),
        4, new CoralAutoToReefCommandGroup(
            swerveDrive, elevatorSubsystem, coralShooterSubsystem, 4, false, false));

    // switch coral and algae mode on button 4
    controlPanel.button(4).and(controlPanel.button(9))
        .whileTrue(Commands.select(coralLeftMap, () -> targetFloor.get()));
    controlPanel.button(4).and(controlPanel.button(9).negate())
        .whileTrue(new SequentialCommandGroup(
            algaeIntakeSubsystem.toAlgaeIntakeDegreeCmd(),
            algaeIntakeSubsystem.intakeCmd()));

    // switch coral and algae mode on button 5
    controlPanel.button(5).and(controlPanel.button(9))
        .whileTrue(Commands.select(coralRightMap, () -> targetFloor.get()));
    controlPanel.button(5).and(controlPanel.button(9).negate())
        .whileTrue(algaeIntakeSubsystem.reverseIntakeCmd());

    // Elastic
    controlPanel.button(4)
        .onTrue(Commands.runOnce(() -> Elastic.selectTab("Limelight")))
        .onFalse(Commands.runOnce(() -> Elastic.selectTab("main")));

    controlPanel.button(5)
        .onTrue(Commands.runOnce(() -> Elastic.selectTab("Limelight")))
        .onFalse(Commands.runOnce(() -> Elastic.selectTab("main")));

    controlPanel.button(7)
        .onTrue(Commands.runOnce(() -> Elastic.selectTab("Limelight")))
        .onFalse(Commands.runOnce(() -> Elastic.selectTab("main")));
  }

  private Command setTargetFloor(int floor) {
    return Commands.runOnce(() -> {
      this.targetFloor = () -> floor;
      SmartDashboard.putNumber("TargetFloor", floor);
    });
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();

  }

  public void autoInit() {
    swerveDrive.resetGyro();
  }

  public Rev2mDistanceSensor getDistanceSensor() {
    return distanceSensor;
  }
}
