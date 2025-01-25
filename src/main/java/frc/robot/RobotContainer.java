// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.RampSubsystem;

public class RobotContainer {
  private final ClimberSubsystem climberSubsystem;
  private final RampSubsystem rampSubsystem;
  private final AlgaeIntakeSubsystem algaeIntakeSubsystem;
  private final SendableChooser<Command> autChooser;

  public RobotContainer() {
    climberSubsystem = new ClimberSubsystem();
    rampSubsystem = new RampSubsystem();
    algaeIntakeSubsystem = new AlgaeIntakeSubsystem();
    autChooser = AutoBuilder.buildAutoChooser();
    autChooser.setDefaultOption("Donothing", Commands.none());
    SmartDashboard.putData("AutoChooser", autChooser);
    SmartDashboard.putData("ALGAElntakeSubsystem", algaeIntakeSubsystem);
    SmartDashboard.putData("RampSubsystem", rampSubsystem);
    SmartDashboard.putData("ClimberSubsystem", climberSubsystem);

    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {

    return Commands.print("No autonomous command configured");
  }
}
