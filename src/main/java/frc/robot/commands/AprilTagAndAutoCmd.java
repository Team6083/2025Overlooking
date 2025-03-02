// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.drivebase.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AprilTagAndAutoCmd extends Command {
  /** Creates a new AprilTagAndAutoCmd. */
  SwerveDrive swerveDrive;
  PIDController driveController = new PIDController(0.1, 0, 0);

  public AprilTagAndAutoCmd(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
    addRequirements(swerveDrive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveController.setSetpoint(swerveDrive.frontLeft.getDriveDistance().in(Meters) + 0.15);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveDrive.drive(
        driveController.calculate(swerveDrive.frontLeft.getDriveDistance().in(Meters)),
        0, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveController.getError() < 0.1;
  }
}
