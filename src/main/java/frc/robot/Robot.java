// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.Elastic;
import frc.robot.lib.TagTracking;

public class Robot extends TimedRobot {
  // CHECKSTYLE.OFF: MemberName
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  // CHECKSTYLE.ON: MemberName

  private boolean saveLogs = false;

  private Timer gcTimer = new Timer();

  private TagTracking tagTracking;

  private Alert limelightAlert;

  public Robot() {
    m_robotContainer = new RobotContainer();
    tagTracking = new TagTracking();
    limelightAlert = new Alert("the position of Limelight is wrong.", AlertType.kWarning);

    CameraServer.startAutomaticCapture();

    gcTimer.start();
    
    SmartDashboard.putBoolean("DisableRightLimelight", false);
    SmartDashboard.putBoolean("DisableLeftLimelight", false);
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

    SmartDashboard.putString("GitInfo", String.format("%s (%s), %s",
        BuildConstants.GIT_SHA,
        BuildConstants.GIT_BRANCH,
        BuildConstants.DIRTY == 1 ? "Dirty" : "Clean"));
    SmartDashboard.putString("BuildDate", BuildConstants.BUILD_DATE);

    Elastic.sendNotification("Robot init", "robot init");
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if (gcTimer.advanceIfElapsed(5)) {
      System.gc();
    }

    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

    if (Math.abs(
        tagTracking.getRightTargetPoseRobotSpace()[0] - tagTracking.getLeftTargetPoseRobotSpace()[0]) > 0.2) {
      limelightAlert.set(true);
    } else {
      limelightAlert.set(false);
    }

    tagTracking.enableDisableLimelight(
        SmartDashboard.getBoolean("DisableRightLimelight", false),
        SmartDashboard.getBoolean("DisableLeftLimelight", false));
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
    m_robotContainer.autoInit();

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
