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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AprilTagAndAutoCmd;
import frc.robot.commands.CoralShooterHoldCmd;
import frc.robot.commands.CoralShooterInWithAutoStopCmd;
import frc.robot.commands.SwerveControlCmd;
import frc.robot.commands.SwerveToTagCmd;
import frc.robot.drivebase.SwerveDrive;
import frc.robot.lib.PowerDistribution;
import frc.robot.lib.TagTracking;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.CoralShooterSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {
  private final PowerDistribution powerDistribution;
  private final CoralShooterSubsystem coralShooterSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final AlgaeIntakeSubsystem algaeIntakeSubsystem;
  private final SwerveDrive swerveDrive;
  private final CommandXboxController mainController;
  private final CommandXboxController viceController;

  private final SendableChooser<Command> autoChooser;

  private final TagTracking tagTracking;

  private final SequentialCommandGroup takeL2AlgaeCommandGroup;
  private final SequentialCommandGroup takeL3AlgaeCommandGroup;

  public RobotContainer() {
    powerDistribution = new PowerDistribution();
    coralShooterSubsystem = new CoralShooterSubsystem(powerDistribution);
    elevatorSubsystem = new ElevatorSubsystem();
    algaeIntakeSubsystem = new AlgaeIntakeSubsystem(powerDistribution);
    swerveDrive = new SwerveDrive();
    mainController = new CommandXboxController(0);

    viceController = new CommandXboxController(1);
    
    tagTracking = new TagTracking();


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
        swerveDrive.setTurningDegreeCmd(0).withTimeout(0.1));

    // NamedCommands.registerCommand("CoralShooterIn",
    // new SequentialCommandGroup(
    // new CoralShooterInWithAutoStopCmd(coralShooterSubsystem),
    // coralShooterSubsystem.coralShooterOnCmd().withTimeout(0.029)));

    NamedCommands.registerCommand("CoralShooterIn",
        coralShooterSubsystem.coralShooterInCmd().withTimeout(0.5));

    NamedCommands.registerCommand("CoralShooterWithStop",
        coralShooterSubsystem.coralShooterOutCmd().withTimeout(1));

    NamedCommands.registerCommand("ErToSec",
        elevatorSubsystem.toSecFloorCmd());

    NamedCommands.registerCommand("ErToTrd",
        elevatorSubsystem.toTrdFloorCmd());

    NamedCommands.registerCommand("ErToFour",
        elevatorSubsystem.toTopFloorCmd());

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
        algaeIntakeSubsystem.setIntakeMotorFastOnCmd());

    NamedCommands.registerCommand("MagicCoral",
        algaeIntakeSubsystem.reIntakeCmd().withTimeout(1));

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
  }

  private void configureBindings() {
    // SwerveDrive
    swerveDrive.setDefaultCommand(new SwerveControlCmd(swerveDrive, mainController));
    // used LeftBumper to switch between fast and slow mode
    mainController.back().onTrue(swerveDrive.gyroResetCmd());

    // CoralShooter
    coralShooterSubsystem.setDefaultCommand(
        new CoralShooterHoldCmd(coralShooterSubsystem));
    mainController.rightBumper().whileTrue(coralShooterSubsystem.coralShooterOutCmd());
    mainController.rightBumper().and(mainController.leftBumper())
        .toggleOnTrue(new CoralShooterInWithAutoStopCmd(coralShooterSubsystem));

    // Elevator
    mainController.povUp().whileTrue(elevatorSubsystem.toTrdFloorCmd());
    mainController.povLeft().whileTrue(elevatorSubsystem.toSecFloorCmd());
    mainController.povDown().whileTrue(elevatorSubsystem.toDefaultPositionCmd());
    mainController.povRight().whileTrue(elevatorSubsystem.toTopFloorCmd());
    mainController.leftTrigger()
        .whileTrue(Commands.either(
            elevatorSubsystem.moveDownCmd(),
            elevatorSubsystem.manualMoveDownCmd(),
            mainController.y()));
    mainController.rightTrigger()
        .whileTrue(Commands.either(
            elevatorSubsystem.moveUpCmd(),
            elevatorSubsystem.manualMoveUpCmd(),
            mainController.y()));
    mainController.start().onTrue(elevatorSubsystem.elevatorReset());

    // ALgaeIntake
    viceController.povUp().whileTrue(algaeIntakeSubsystem.manualRotateUpCmd());
    viceController.povDown().whileTrue(algaeIntakeSubsystem.manualRotateDownCmd());
    viceController.povRight().whileTrue(algaeIntakeSubsystem.toDefaultDegreeCmd());
    viceController.povLeft().whileTrue(algaeIntakeSubsystem.toAlgaeIntakeDegreeCmd());
    viceController.b().whileTrue(algaeIntakeSubsystem.reIntakeCmd());
    viceController.x().whileTrue(algaeIntakeSubsystem.setIntakeMotorFastOnCmd());

    // Elevator + AlgaeIntake
    viceController.y().toggleOnTrue(takeL2AlgaeCommandGroup);
    viceController.a().toggleOnTrue(takeL3AlgaeCommandGroup);

    // TagTracking



    mainController.x().whileTrue(new SwerveToTagCmd(swerveDrive, false));
    mainController.b().whileTrue(new SwerveToTagCmd(swerveDrive, true));

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
