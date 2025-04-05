// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class TagTracking {
  private final NetworkTable leftTable;
  private final NetworkTable rightTable;

  public TagTracking() {
    leftTable = NetworkTableInstance.getDefault().getTable("limelight-left");
    rightTable = NetworkTableInstance.getDefault().getTable("limelight-right");
    setLedMode(0);
    setPipeline(0);
  }

  private void setLedMode(int ledMode) {
    rightTable.getEntry("ledMode").setNumber(ledMode);
    leftTable.getEntry("ledMode").setNumber(ledMode);
  }

  private void setPipeline(int pipeline) {
    rightTable.getEntry("pipeline").setNumber(pipeline);
    leftTable.getEntry("pipeline").setNumber(pipeline);
  }

  // tv int 1 if valid target exists. 0 if no valid targets exist.
  public double getRightTv() {
    return rightTable.getEntry("tv").getDouble(0);
  }

  public double getLeftTv() {
    return leftTable.getEntry("tv").getDouble(0);
  }

  public Boolean isGetTarget() {
    return getRightTv() == 1 || getLeftTv() == 1;
  }

  public double getRightTid() {
    if (getRightTv() == 1) {
      double id = rightTable.getEntry("tid").getDouble(0);
      return id;
    } else {
      return 0;
    }
  }

  public double getLeftTid() {
    if (getLeftTv() == 1) {
      double id = leftTable.getEntry("tid").getDouble(0);
      return id;
    } else {
      return 0;
    }
  }

  public double getBestTargetId() {
    if (getRightTv() == 1) {
      return getRightTid();
    } else if (getLeftTv() == 1) {
      return getLeftTid();
    } else {
      return 0;
    }
  }

  // [tx, ty, tz, pitch, yaw, roll] (meters, degrees)
  public double[] getRightTargetPoseRobotSpace() {
    return rightTable.getEntry("targetPose_robotSpace").getDoubleArray(new double[6]);
  }

  public double[] getLeftTargetPoseRobotSpace() {
    return leftTable.getEntry("targetPose_robotSpace").getDoubleArray(new double[6]);
  }

  public double get3DTx() {
    if (getRightTv() == 1 && getLeftTv() == 1 && getRightTid() == getLeftTid()) {
      return (getRightTargetPoseRobotSpace()[0] + getLeftTargetPoseRobotSpace()[0]) / 2;
    } else if (getRightTv() == 1) {
      return getRightTargetPoseRobotSpace()[0];
    } else if (getLeftTv() == 1) {
      return getLeftTargetPoseRobotSpace()[0];
    } else {
      return 0;
    }
  }

  public double get3DTz() {
    if (getRightTv() == 1 && getLeftTv() == 1 && getRightTid() == getLeftTid()) {
      return (getRightTargetPoseRobotSpace()[2] + getLeftTargetPoseRobotSpace()[2]) / 2;
    } else if (getRightTv() == 1) {
      return getRightTargetPoseRobotSpace()[2];
    } else if (getLeftTv() == 1) {
      return getLeftTargetPoseRobotSpace()[2];
    } else {
      return 0;
    }
  }

  public double get3DYaw() {
    if (getRightTv() == 1 && getLeftTv() == 1 && getRightTid() == getLeftTid()) {
      return (getRightTargetPoseRobotSpace()[4] + getLeftTargetPoseRobotSpace()[4]) / 2;
    } else if (getRightTv() == 1) {
      return getRightTargetPoseRobotSpace()[4];
    } else if (getLeftTv() == 1) {
      return getLeftTargetPoseRobotSpace()[4];
    } else {
      return 0;
    }
  }
}
