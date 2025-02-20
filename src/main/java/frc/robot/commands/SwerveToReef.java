// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.drivebase.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveToReef extends Command {
  /** Creates a new SwerveToReef. */
  SwerveDrive swerveDrive;
  PIDController ySpeedController;

  public SwerveToReef(SwerveDrive swerveDrive) {
    ySpeedController = new PIDController(0.01, 0, 0);
    this.swerveDrive = swerveDrive;
    addRequirements(swerveDrive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double ySetpoint = swerveDrive.getPose2d().getMeasureY()
        .plus(Inches.of(6.5)).in(Meter);
    ySpeedController.setSetpoint(ySetpoint);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = ySpeedController.calculate(swerveDrive.getPose2d().getMeasureY().in(Meter));
    output = MathUtil.clamp(output, 0.1, 0.1);
    swerveDrive.drive(0, output, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ySpeedController.getError() < 0.01;
  }
}
