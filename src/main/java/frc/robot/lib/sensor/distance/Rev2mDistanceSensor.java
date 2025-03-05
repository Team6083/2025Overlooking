package frc.robot.lib.sensor.distance;

import com.revrobotics.Rev2mDistanceSensor.Port;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class Rev2mDistanceSensor implements DistanceSensor, AutoCloseable, Sendable {

  private final com.revrobotics.Rev2mDistanceSensor actualSensor;

  private final SimDevice simSensor;
  private final SimDouble simDistance;

  public Rev2mDistanceSensor(Port port) {
    simSensor = SimDevice.create("Sensor:Rev2mDistanceSensor", 0);
    if (simSensor != null) {
      simDistance = simSensor.createDouble("distance", Direction.kInput, 0);
      actualSensor = null;
    } else {
      simDistance = null;
      actualSensor = new com.revrobotics.Rev2mDistanceSensor(port);
    }
  }

  public com.revrobotics.Rev2mDistanceSensor getActualSensor() {
    return actualSensor;
  }

  @Override
  public double getRange() {
    if (simDistance != null) {
      return simDistance.get();
    }
    return actualSensor.getRange();
  }

  @Override
  public boolean isRangeValid() {
    if (simDistance != null) {
      return true;
    }
    return actualSensor.isRangeValid();
  }

  @Override
  public void setAutomaticMode(boolean modeOn){
    actualSensor.setAutomaticMode(modeOn);
  }

  @Override
  public void close() {
    simSensor.close();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Distance", this::getRange, null);
    builder.addBooleanProperty("DistanceValid", this::isRangeValid, null);
  }

}