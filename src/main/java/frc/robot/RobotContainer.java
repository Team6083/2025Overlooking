package frc.robot;

// WPILib imports
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// PathPlanner imports
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

// Robot-specific imports
import frc.robot.commands.CoralShooterHoldCmd;
import frc.robot.commands.CoralShooterInWithAutoStopCmd;
import frc.robot.commands.SwerveControlCmd;
import frc.robot.drivebase.SwerveDrive;
import frc.robot.lib.TagTracking;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.CoralShooterSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// Java imports
import java.util.function.Supplier;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (including the scheduler calls). Instead, the structure of
 * the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class RobotContainer {
  // Subsystems
  private final CoralShooterSubsystem coralShooterSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final AlgaeIntakeSubsystem algaeIntakeSubsystem;
  private final SwerveDrive swerveDrive;

  // Controllers
  private final CommandXboxController mainController = new CommandXboxController(0);
  private final CommandGenericHID controlPanel = new CommandGenericHID(1);

  // Supplier functions for control panel buttons
  private final Supplier<Boolean> elevatorBypassSafety = () -> controlPanel.button(9).getAsBoolean();

  // Autonomous mode selection
  private final SendableChooser<Command> autoChooser;

  // Vision tracking
  private final TagTracking tagTracking;

  // Command groups for complex operations
  private final SequentialCommandGroup takeL2AlgaeCommandGroup;
  private final SequentialCommandGroup takeL3AlgaeCommandGroup;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Initialize control panel button suppliers
    Supplier<Boolean> elevatorUsePID = () -> controlPanel.button(10).getAsBoolean();
    Supplier<Boolean> algaeRotateUsePID = () -> controlPanel.button(12).getAsBoolean();

    // Initialize subsystems
    coralShooterSubsystem = new CoralShooterSubsystem();
    elevatorSubsystem = new ElevatorSubsystem(elevatorUsePID, elevatorBypassSafety);
    algaeIntakeSubsystem = new AlgaeIntakeSubsystem(algaeRotateUsePID);
    swerveDrive = new SwerveDrive();
    tagTracking = new TagTracking();

    // Initialize command groups
    takeL2AlgaeCommandGroup = createTakeL2AlgaeCommandGroup();
    takeL3AlgaeCommandGroup = createTakeL3AlgaeCommandGroup();

    // Register named commands for autonomous mode
    registerNamedCommands();

    // Configure autonomous mode selection
    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    SmartDashboard.putData("AutoChooser", autoChooser);

    // Add subsystems to SmartDashboard for monitoring
    addSubsystemsToDashboard();

    // Configure button bindings
    configureBindings();
  }

  /**
   * Creates the command group for taking algae from level 2.
   * 
   * @return The command group for level 2 algae collection
   */
  private SequentialCommandGroup createTakeL2AlgaeCommandGroup() {
    return new ParallelRaceGroup(
        elevatorSubsystem.toGetSecAlgaeCmd().repeatedly()
            .until(() -> elevatorSubsystem.getAbsoluteError() < 5),
        algaeIntakeSubsystem.reverseIntakeCmd())
        .andThen(new ParallelRaceGroup(
            new RunCommand(() -> swerveDrive.drive(-0.4, 0, 0, false), swerveDrive)
                .withTimeout(1.5),
            algaeIntakeSubsystem.reverseIntakeCmd()));
  }

  /**
   * Creates the command group for taking algae from level 3.
   * 
   * @return The command group for level 3 algae collection
   */
  private SequentialCommandGroup createTakeL3AlgaeCommandGroup() {
    return new ParallelRaceGroup(
        elevatorSubsystem.toGetTrdAlgaeCmd().repeatedly()
            .until(() -> elevatorSubsystem.getAbsoluteError() < 5),
        algaeIntakeSubsystem.reverseIntakeCmd())
        .andThen(new ParallelRaceGroup(
            new RunCommand(() -> swerveDrive.drive(-0.4, 0, 0, false), swerveDrive)
                .withTimeout(1.5),
            algaeIntakeSubsystem.reverseIntakeCmd()));
  }

  /**
   * Adds all subsystems to the SmartDashboard for monitoring.
   */
  private void addSubsystemsToDashboard() {
    SmartDashboard.putData("CoralShooterSubsystem", coralShooterSubsystem);
    SmartDashboard.putData("ElevatorSubsystem", elevatorSubsystem);
    SmartDashboard.putData("AlgaeIntakeSubsystem", algaeIntakeSubsystem);
    SmartDashboard.putData("SwerveDrive", swerveDrive);
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
    //     Commands.either(new SwerveToTagCmd(swerveDrive, false).withTimeout(4),
    //         swerveDrive.driveForwardCmd().withTimeout(2),
    //         () -> tagTracking.getTv() == 1));

    // NamedCommands.registerCommand("AprilTagLeft",
    //     Commands.either(new SwerveToTagCmd(swerveDrive, true).withTimeout(4),
    //         swerveDrive.driveForwardCmd().withTimeout(2),
    //         () -> tagTracking.getTv() == 1));

    NamedCommands.registerCommand("AlgaeIntake",
        algaeIntakeSubsystem.intakeCmd());

    NamedCommands.registerCommand("CoralShooterHold",
        new CoralShooterHoldCmd(coralShooterSubsystem));
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
    controlPanel.button(8).whileTrue(algaeIntakeSubsystem.toAlgaeIntakeDegreeCmd());
    controlPanel.button(3).whileTrue(algaeIntakeSubsystem.reverseIntakeCmd());
    controlPanel.button(1).whileTrue(algaeIntakeSubsystem.intakeCmd());
    mainController.x().whileTrue(algaeIntakeSubsystem.intakeCmd());
    mainController.b().whileTrue(algaeIntakeSubsystem.reverseIntakeCmd());

    // Elevator + AlgaeIntake
    controlPanel.button(6).whileTrue(takeL2AlgaeCommandGroup);
    controlPanel.button(5).whileTrue(takeL3AlgaeCommandGroup);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
