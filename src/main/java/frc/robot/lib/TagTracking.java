// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.io.IOException;
import java.util.Optional;

/** Add your docs here. */
public class TagTracking {
  private final NetworkTable leftTable;
  private final NetworkTable rightTable;
  private final AprilTagFieldLayout layout;

  public TagTracking() {
    // TODO: limelight name
    leftTable = NetworkTableInstance.getDefault().getTable("limelight-left");
    rightTable = NetworkTableInstance.getDefault().getTable("limelight-right");
    setLedMode(0);
    setPipeline(0);
    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
    } catch (IOException err) {
      throw new RuntimeException();
    }
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

  public double[] getRightTargetposeRobotspace() {
    return rightTable.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
  }

  public double[] getLeftTargetposeRobotspace() {
    return leftTable.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
  }

  public double get3DTx() {
    if (getRightTv() == 1) {
      return getRightTargetposeRobotspace()[0];
    } else if (getLeftTv() == 1) {
      return getLeftTargetposeRobotspace()[0];
    } else if (getLeftTv() == 1 && getRightTv() == 1) {
      return (getRightTargetposeRobotspace()[0] + getLeftTargetposeRobotspace()[0]) / 2;
    } else {
      return 0;
    }
  }

  // [tx, ty, tz, pitch, yaw, roll] (meters, degrees)
  public double get3DTy() {
    if (getRightTv() == 1) {
      return getRightTargetposeRobotspace()[0];
    } else if (getLeftTv() == 1) {
      return getLeftTargetposeRobotspace()[0];
    } else if (getLeftTv() == 1 && getRightTv() == 1) {
      return (getRightTargetposeRobotspace()[1] + getLeftTargetposeRobotspace()[1]) / 2;
    } else {
      return 0;
    }
  }

  public double get3DTz() {
    if (getRightTv() == 1) {
      return getRightTargetposeRobotspace()[0];
    } else if (getLeftTv() == 1) {
      return getLeftTargetposeRobotspace()[0];
    } else if (getLeftTv() == 1 && getRightTv() == 1) {
      return (getRightTargetposeRobotspace()[2] + getLeftTargetposeRobotspace()[2]) / 2;
    } else {
      return 0;
    }
  }

  public double get3DPitch() {
    if (getRightTv() == 1) {
      return getRightTargetposeRobotspace()[0];
    } else if (getLeftTv() == 1) {
      return getLeftTargetposeRobotspace()[0];
    } else if (getLeftTv() == 1 && getRightTv() == 1) {
      return (getRightTargetposeRobotspace()[3] + getLeftTargetposeRobotspace()[3]) / 2;
    } else {
      return 0;
    }
  }

  public double get3DYaw() {
    if (getRightTv() == 1) {
      return getRightTargetposeRobotspace()[0];
    } else if (getLeftTv() == 1) {
      return getLeftTargetposeRobotspace()[0];
    } else if (getLeftTv() == 1 && getRightTv() == 1) {
      return (getRightTargetposeRobotspace()[4] + getLeftTargetposeRobotspace()[4]) / 2;
    } else {
      return 0;
    }
  }

  public double get3DRoll() {
    if (getRightTv() == 1) {
      return getRightTargetposeRobotspace()[0];
    } else if (getLeftTv() == 1) {
      return getLeftTargetposeRobotspace()[0];
    } else if (getLeftTv() == 1 && getRightTv() == 1) {
      return (getRightTargetposeRobotspace()[5] + getLeftTargetposeRobotspace()[5]) / 2;
    } else {
      return 0;
    }
  }

  // public double getTv() {
  // double tv = table.getEntry("tv").getDouble(0);
  // return tv;
  // }

  // public double getTid() {
  // double id = table.getEntry("tid").getDouble(0);
  // return id;
  // }

  // public double[] getBt() {
  // double[] bt = table.getEntry("botpose_targetspace").getDoubleArray(new
  // double[6]);
  // return bt;
  // }

  // public double[] getTr() {
  // double[] tr = table.getEntry("targetpose_robotspace").getDoubleArray(new
  // double[6]);
  // return tr;
  // }

  // public double getDistance() {
  // // readValue();
  // // CHECKSTYLE.OFF: LocalVariableName
  // double targetHeight = getBt()[1]; // botpose in targetspace y
  // double xDis = getBt()[0];
  // double zDis = getBt()[2];
  // double horDis = Math.sqrt(Math.pow(xDis, 2) + Math.pow(zDis, 2));
  // double distance = Math.sqrt(Math.pow(targetHeight, 2) + Math.pow(horDis, 2));
  // return distance;
  // // CHECKSTYLE.ON: LocalVariableName
  // }

  // public Pose2d getTagPose2d() {
  // return getTagPose3d().toPose2d();
  // }

  // public Pose3d getTagPose3d() {
  // return getDesiredTagPose3d(getTid());
  // }

  // public Pose3d getDesiredTagPose3d(double index) {
  // if (getTv() == 1) {
  // Optional<Pose3d> tagPose3d = layout.getTagPose((int) index);
  // Pose3d tagPose = tagPose3d.isPresent() ? tagPose3d.get() : new Pose3d();
  // return tagPose;
  // } else {
  // return new Pose3d();
  // }
  // }

  // public Pose2d getDesiredTagPose2d(double index) {
  // return getDesiredTagPose3d(index).toPose2d();
  // }
}
