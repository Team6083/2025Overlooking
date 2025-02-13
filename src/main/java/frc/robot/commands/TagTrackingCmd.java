// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.vision.TagTracking;
import frc.robot.drivebase.SwerveDrive;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TagTrackingCmd extends Command {
  /** Creates a new TagTrackingCmd. */
  private final TagTracking tagTracking;
  private final SwerveDrive SwerveDrive;
  private double modification1 =-0.5;
  private double modification2 =-0.3;

  public TagTrackingCmd(TagTracking tagTracking,SwerveDrive SwerveDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.tagTracking = tagTracking;
    this.SwerveDrive = SwerveDrive;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   if (tagTracking.getTv() == 1) {
      SwerveDrive.drive(tagTracking.getTy()*modification2, tagTracking.getTx()*modification1, 0, false);
   }
   if (tagTracking.getTy() >= -2) {
    SwerveDrive.drive(0, 0, 0, false);
   }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
