package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.io.IOException;
import java.util.Optional;

public class TagTracking {
  private final NetworkTable table;
  private final AprilTagFieldLayout layout;

  public TagTracking() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    setLedMode(0);
    setPipeline(0);
    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException err) {
      throw new RuntimeException();
    }
  }

  private void setLedMode(int ledMode) {
    table.getEntry("ledMode").setNumber(ledMode);
  }

  private void setPipeline(int pipeline) {
    table.getEntry("pipeline").setNumber(pipeline);
  }

  public double[] getCt() {
    double[] ct = table.getEntry("camtran").getDoubleArray(new double[6]);
    return ct;
  }

  public double getTx() {
    double tx = table.getEntry("tx").getDouble(0);
    return tx;
  }

  public double getTy() {
    double ty = table.getEntry("ty").getDouble(0);
    return ty;
  }

  public double getTv() {
    double tv = table.getEntry("tv").getDouble(0);
    return tv;
  }

  public double getTid() {
    double id = table.getEntry("tid").getDouble(0);
    return id;
  }

  public double[] getBt() {
    double[] bt = table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
    return bt;
  }

  public double getDistance() {
    // readValue();
    // CHECKSTYLE.OFF: LocalVariableName
    double targetHeight = getBt()[1]; // botpose in targetspace y
    double xDis = getBt()[0];
    double zDis = getBt()[2];
    double horDis = Math.sqrt(Math.pow(xDis, 2) + Math.pow(zDis, 2));
    double distance = Math.sqrt(Math.pow(targetHeight, 2) + Math.pow(horDis, 2));
    return distance;
    // CHECKSTYLE.ON: LocalVariableName
  }
  

  public Pose2d getTagPose2d() {
    getTagPose3d().toPose2d();
  }

  public Pose3d getTagPose3d() {
    if (getTv() == 1) {
      Optional<Pose3d> tagPose3d = layout.getTagPose((int) getTid());
      Pose3d tagPose = tagPose3d.isPresent() ? tagPose3d.get() : new Pose3d();
      return tagPose;
    } else {
      return new Pose3d();
    }
  }

  public Pose2d getDesiredTagPose2d(double index) {
    if (getTv() == 1) {
      Optional<Pose3d> tagPose3d = layout.getTagPose((int) index);
      Pose2d tagPose2d = tagPose3d.isPresent() ? tagPose3d.get().toPose2d() : new Pose2d();
      return tagPose2d;
    } else {
      return new Pose2d();
    }
  }
}
