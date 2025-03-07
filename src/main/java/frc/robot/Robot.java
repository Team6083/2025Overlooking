// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  // CHECKSTYLE.OFF: MemberName
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  // CHECKSTYLE.ON: MemberName

  private boolean saveLogs = true;
  Timer gcTimer = new Timer();

  public Robot() {
    m_robotContainer = new RobotContainer();
    CameraServer.startAutomaticCapture();
    gcTimer.start();
  }

  @Override
  public void robotInit() {
    if (saveLogs) {
      DataLogManager.start();
      DriverStation.startDataLog(DataLogManager.getLog());
    }

    NetworkTableInstance.getDefault().getStringTopic("/Metadata/BuildDate").publish()
        .set(BuildConstants.BUILD_DATE);
    NetworkTableInstance.getDefault().getStringTopic("/Metadata/GitBranch").publish()
        .set(BuildConstants.GIT_BRANCH);
    NetworkTableInstance.getDefault().getStringTopic("/Metadata/GitDate").publish()
        .set(BuildConstants.GIT_DATE);
    NetworkTableInstance.getDefault().getStringTopic("/Metadata/GitDirty").publish()
        .set(BuildConstants.DIRTY == 1 ? "Dirty!" : "Clean! Good job!");
    NetworkTableInstance.getDefault().getStringTopic("/Metadata/GitSHA").publish()
        .set(BuildConstants.GIT_SHA);
    NetworkTableInstance.getDefault().getStringTopic("/Metadata/GitBranch").publish()
        .set(BuildConstants.GIT_BRANCH);

    SmartDashboard.putString("Maven_Name", BuildConstants.MAVEN_NAME);
    SmartDashboard.putString("Version", BuildConstants.VERSION);
    SmartDashboard.putString("Git_SHA", BuildConstants.GIT_SHA);
    SmartDashboard.putString("Git_DATE", BuildConstants.GIT_DATE);
    SmartDashboard.putString("Git_Branch", BuildConstants.GIT_BRANCH);
    SmartDashboard.putString("Build_Date", BuildConstants.BUILD_DATE);
    SmartDashboard.putString("Git_Branch", BuildConstants.GIT_BRANCH);
    if (BuildConstants.DIRTY == 0) {
      SmartDashboard.putString(
          "Dirty", "Clean! Good job!");
    } else {
      SmartDashboard.putString(
          "Dirty", "Dirty!");
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if (gcTimer.advanceIfElapsed(5)) {
      System.gc();
    }
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
