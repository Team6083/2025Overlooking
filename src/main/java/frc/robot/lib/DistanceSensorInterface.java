package frc.robot.lib;

public interface DistanceSensorInterface {
  public double getTargetDistance();

  public boolean isGetTarget();

  public void setAutomaticMode(boolean isEnable);
}