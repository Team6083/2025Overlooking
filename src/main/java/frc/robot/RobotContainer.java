package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoCoralAndElevatorCmd;
import frc.robot.commands.CoralShooterHoldCmd;
import frc.robot.commands.CoralShooterInWithAutoStopCmd;
import frc.robot.commands.SwerveControlCmd;
import frc.robot.commands.TakeAlgaeCommandGroup;
import frc.robot.drivebase.SwerveDrive;
import frc.robot.lib.Elastic;
import frc.robot.lib.Elastic.Notification.NotificationLevel;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.CoralShooterSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.RgbLedSubsystem;
import java.util.Map;
import java.util.function.Supplier;

public class RobotContainer {
  private final CoralShooterSubsystem coralShooterSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final AlgaeIntakeSubsystem algaeIntakeSubsystem;
  private final RgbLedSubsystem rgbLedSubsystem;
  private final SwerveDrive swerveDrive;

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
    rgbLedSubsystem = new RgbLedSubsystem(coralShooterSubsystem);
    swerveDrive = new SwerveDrive();

    registerNamedCommands();

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    SmartDashboard.putData("AutoChooser", autoChooser);

    SmartDashboard.putData("CoralShooterSubsystem", coralShooterSubsystem);
    SmartDashboard.putData("ElevatorSubsystem", elevatorSubsystem);
    SmartDashboard.putData("AlgaeIntakeSubsystem", algaeIntakeSubsystem);
    SmartDashboard.putData("SwerveDrive", swerveDrive);

    SmartDashboard.putNumber("targetFloor", 2);

    configureBindings();
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("SetTurningDegree",
        swerveDrive.setTurningDegreeCmd(0).withTimeout(0.0000001));

    NamedCommands.registerCommand("CoralIn",
        new CoralShooterInWithAutoStopCmd(coralShooterSubsystem)
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

    NamedCommands.registerCommand("AlgaeIntake",
        algaeIntakeSubsystem.intakeCmd());

    NamedCommands.registerCommand("CoralShooterHold",
        new CoralShooterHoldCmd(coralShooterSubsystem));

    NamedCommands.registerCommand("CoralLeftL2",
        new AutoCoralAndElevatorCmd(
            swerveDrive, elevatorSubsystem, coralShooterSubsystem, 2, true, true));

    NamedCommands.registerCommand("CoralRightL2",
        new AutoCoralAndElevatorCmd(
            swerveDrive, elevatorSubsystem, coralShooterSubsystem, 2, false, true));

    NamedCommands.registerCommand("CoralLeftL4",
        new AutoCoralAndElevatorCmd(
            swerveDrive, elevatorSubsystem, coralShooterSubsystem, 4, true, true));

    NamedCommands.registerCommand("CoralRightL4",
        new AutoCoralAndElevatorCmd(
            swerveDrive, elevatorSubsystem, coralShooterSubsystem, 4, false, true));

    NamedCommands.registerCommand("TakeAlgae",
        new TakeAlgaeCommandGroup(
            swerveDrive, elevatorSubsystem, algaeIntakeSubsystem, 2));

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
        .toggleOnTrue(new SequentialCommandGroup(new CoralShooterInWithAutoStopCmd(coralShooterSubsystem),
            coralShooterSubsystem.coralShooterInCmd()
                .withTimeout(ConfigChooser.CoralShooter.getDouble("kCoralInTimeOut")),
            rgbLedSubsystem.setLightBlinkCmd(6, 100)));));
    controlPanel.button(6)
        .toggleOnTrue(new SequentialCommandGroup(new CoralShooterInWithAutoStopCmd(coralShooterSubsystem),
            coralShooterSubsystem.coralShooterInCmd()
                .withTimeout(ConfigChooser.CoralShooter.getDouble("kCoralInTimeOut"))));

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
    controlPanel.button(3).onTrue(new ParallelCommandGroup(
        setTargetFloor(2).andThen(Commands.runOnce(() -> elasticNotification("Floor Changed", "Floor 2 selected"))),
        rgbLedSubsystem.setLightBlinkCmd(2, 200));
    controlPanel.button(2).onTrue(new ParallelCommandGroup(
        setTargetFloor(3).andThen(Commands.runOnce(() -> elasticNotification("Floor Changed", "Floor 2 selected"))),
        rgbLedSubsystem.setLightBlinkCmd(2, 200));
    controlPanel.button(1).onTrue(new ParallelCommandGroup(
        setTargetFloor(4).andThen(Commands.runOnce(() -> elasticNotification("Floor Changed", "Floor 2 selected"))),
        rgbLedSubsystem.setLightBlinkCmd(2, 200));

    Map<Integer, Command> oneButtonAlgaeMap = Map.of(
        2, new TakeAlgaeCommandGroup(
            swerveDrive, elevatorSubsystem, algaeIntakeSubsystem, 2),
        3, new TakeAlgaeCommandGroup(
            swerveDrive, elevatorSubsystem, algaeIntakeSubsystem, 3));

    controlPanel.button(7).whileTrue(Commands.select(oneButtonAlgaeMap, () -> targetFloor.get()));

    Map<Integer, Command> coralLeftMap = Map.of(
        2, new AutoCoralAndElevatorCmd(
            swerveDrive, elevatorSubsystem, coralShooterSubsystem, 2, true, false),
        3, new AutoCoralAndElevatorCmd(
            swerveDrive, elevatorSubsystem, coralShooterSubsystem, 3, true, false),
        4, new AutoCoralAndElevatorCmd(
            swerveDrive, elevatorSubsystem, coralShooterSubsystem, 4, true, false));

    Map<Integer, Command> coralRightMap = Map.of(
        2, new AutoCoralAndElevatorCmd(
            swerveDrive, elevatorSubsystem, coralShooterSubsystem, 2, false, false),
        3, new AutoCoralAndElevatorCmd(
            swerveDrive, elevatorSubsystem, coralShooterSubsystem, 3, false, false),
        4, new AutoCoralAndElevatorCmd(
            swerveDrive, elevatorSubsystem, coralShooterSubsystem, 4, false, false));

    controlPanel.button(4).whileTrue(
        Commands.either(
            Commands.select(coralLeftMap, () -> targetFloor.get()),
            new SequentialCommandGroup(
                algaeIntakeSubsystem.toAlgaeIntakeDegreeCmd(),
                algaeIntakeSubsystem.intakeCmd()),
            controlPanel.button(9)));

    controlPanel.button(5).whileTrue(
        Commands.either(
            Commands.select(coralRightMap, () -> targetFloor.get()),
            algaeIntakeSubsystem.reverseIntakeCmd(),
            controlPanel.button(9)));

    // Elastic
    new Trigger(controlPanel.button(4)::getAsBoolean)
        .onTrue(Commands.runOnce(() -> Elastic.selectTab("Limelight")))
        .onFalse(Commands.runOnce(() -> Elastic.selectTab("main")));

    new Trigger(controlPanel.button(5)::getAsBoolean)
        .onTrue(Commands.runOnce(() -> Elastic.selectTab("Limelight")))
        .onFalse(Commands.runOnce(() -> Elastic.selectTab("main")));

    new Trigger(controlPanel.button(7)::getAsBoolean)
        .onTrue(Commands.runOnce(() -> Elastic.selectTab("Limelight")))
        .onFalse(Commands.runOnce(() -> Elastic.selectTab("main")));
  }

  private void elasticNotification(String title, String description) {
    Elastic.Notification notification = new Elastic.Notification();
    Elastic.sendNotification(notification
        .withLevel(NotificationLevel.INFO)
        .withTitle(title)
        .withDescription(description)
        .withDisplaySeconds(3.0));
  }

  private Command setTargetFloor(int floor) {
    return Commands.runOnce(() -> {
      this.targetFloor = () -> floor;
      SmartDashboard.putNumber("targetFloor", floor);
    });
  }

  public Command getAutonomousCommand() {
    return Commands.runOnce(() -> swerveDrive.resetGyro())
        .andThen(autoChooser.getSelected());
  }
}
