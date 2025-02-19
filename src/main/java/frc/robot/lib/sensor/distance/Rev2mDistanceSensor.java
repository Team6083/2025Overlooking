package frc.robot.lib.sensor.distance;

public class Rev2mDistanceSensor extends com.revrobotics.Rev2mDistanceSensor implements DistanceSensor {

  public Rev2mDistanceSensor(Port port) {
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