package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralShooterInWithAutoStopCmd extends Command {
  /** Creates a new CoralShooterInWithAutoStopCmd. */

  private final CoralShooterSubsystem coralShooterSubsystem;

  public CoralShooterInWithAutoStopCmd(CoralShooterSubsystem coralShooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.coralShooterSubsystem = coralShooterSubsystem;
    addRequirements(this.coralShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coralShooterSubsystem.coralShooterOut();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralShooterSubsystem.coralShooterStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return coralShooterSubsystem.isGetTarget();
  }
}