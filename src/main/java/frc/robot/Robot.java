// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.Core;
import org.opencv.core.Mat;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
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
  private UsbCamera camera;

  private final RobotContainer m_robotContainer;
  // CHECKSTYLE.ON: MemberName

  private boolean saveLogs = true;
  Timer gcTimer = new Timer();

  public Robot() {
    m_robotContainer = new RobotContainer();

    camera =CameraServer.startAutomaticCapture();
    camera.setResolution(640, 480);
    camera.setFPS(30);

    new Thread (()->{
     CvSink cvSink = CameraServer.getVideo();
      CvSource outputStream = CameraServer.putVideo("Flipped Camera", 640, 400);

      Mat frame = new Mat();

      while(!Thread.interrupted()){

        if ( cvSink.grabFrame(frame)==0){
          continue;
        }

        Core.flip(frame, frame, 0);
        outputStream.putFrame(frame);
      }
    }).start();
    gcTimer.start();
  }

  @Override
  public void robotInit() {
    if (saveLogs) {
      DataLogManager.start();
      DriverStation.startDataLog(DataLogManager.getLog());
    }

    SmartDashboard.putString("Maven_Name", BuildConstants.MAVEN_NAME);
    SmartDashboard.putString("Version", BuildConstants.VERSION);
    SmartDashboard.putString("Git_SHA", BuildConstants.GIT_SHA);
    SmartDashboard.putString("Git_DATE", BuildConstants.GIT_DATE);
    SmartDashboard.putString("Git_Branch", BuildConstants.GIT_BRANCH);
    SmartDashboard.putString("Build_Date", BuildConstants.BUILD_DATE);
    SmartDashboard.putString("Git_Branch", BuildConstants.GIT_BRANCH);
    SmartDashboard.putString("Build_Date", BuildConstants.BUILD_DATE);
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
    if(gcTimer.advanceIfElapsed(5)){
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
