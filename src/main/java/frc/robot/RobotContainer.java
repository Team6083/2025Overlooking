// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.drivebase.SwerveDrive;



public class RobotContainer {
 
  private final SendableChooser<Command> autoChooser;
  private final SwerveDrive swerveDrive;
  private final SwerveJoystickCmd swerveJoystickCmd;
  private final CommandXboxController mainController; 
 
  public RobotContainer() {
    
    swerveDrive = new SwerveDrive();
    mainController = new CommandXboxController(0);
    swerveJoystickCmd = new SwerveJoystickCmd(swerveDrive, mainController);
    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("DoNothing", Commands.none());
    autoChooser.addOption("123456", Commands.none());
   
    SmartDashboard.putData("AutoChooser", autoChooser);
   

    configureBindings();
  }
    

  private void configureBindings() {
    swerveDrive.setDefaultCommand(swerveJoystickCmd);
    mainController.a().whileTrue(swerveDrive.setTurningDegree90Cmd());
    mainController.b().whileTrue(swerveDrive.resetTurningCmd());
    mainController.back().whileTrue(swerveDrive.gyroResetCmd());
  }

  public Command getAutonomousCommand() {
    
    return autoChooser.getSelected();
    }
  }

