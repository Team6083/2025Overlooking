// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  // CHECKSTYLE.OFF: MemberName
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  // CHECKSTYLE.ON: MemberName

  private boolean saveLogs = false;

  public Robot() {
    m_robotContainer = new RobotContainer();
    CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotInit() {
    if (saveLogs) {
      DataLogManager.start();
      DriverStation.startDataLog(DataLogManager.getLog());
    }

    SmartDashboard.putString("MAVEN_NAME", BuildConstants.MAVEN_NAME);
    SmartDashboard.putString("VERSION", BuildConstants.VERSION);
    SmartDashboard.putString("GIT_SHA", BuildConstants.GIT_SHA);
    SmartDashboard.putString("GIT_DATE", BuildConstants.GIT_DATE);
    SmartDashboard.putString("GIT_BRANCH", BuildConstants.GIT_BRANCH);
    SmartDashboard.putString("BUILD_DATE", BuildConstants.BUILD_DATE);
    SmartDashboard.putString("GIT_BRANCH", BuildConstants.GIT_BRANCH);
    SmartDashboard.putString("BUILD_DATE", BuildConstants.BUILD_DATE);
    if (BuildConstants.DIRTY == 0) {
      SmartDashboard.putString(
          "DIRTY", "No uncommitted changes? This broccoli is looking fresh and crispy!");
    } else {
      SmartDashboard.putString(
          "DIRTY", "Your code smells stronger than overcooked broccoli. Maybe it's time to commit?");
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
