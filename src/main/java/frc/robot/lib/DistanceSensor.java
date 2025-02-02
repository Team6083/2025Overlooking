package frc.robot.lib;

import com.revrobotics.Rev2mDistanceSensor;

public class DistanceSensor extends Rev2mDistanceSensor implements DistanceSensorInterface {

  public DistanceSensor(Port port) {
    super(port);
  }

  @Override
  public double getTargetDistance() {
    return super.getRange();
  }

  @Override
  public boolean isGetTarget() {
    return super.isRangeValid();
  }

}