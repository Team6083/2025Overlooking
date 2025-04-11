package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RGBLedSubsystem extends SubsystemBase {
  private Solenoid leftLed;
  private Solenoid rightLed;
  private boolean isInBlink = false;
  private int blinkingFrequency;
  private final CoralShooterSubsystem coralShooterSubsystem;

  public RGBLedSubsystem(CoralShooterSubsystem coralShooterSubsystem) {
    this.coralShooterSubsystem = coralShooterSubsystem;

    leftLed = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    rightLed = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
  }

  public void turnOn() {
    leftLed.set(true);
    rightLed.set(true);
  }

  public void turnOff() {
    leftLed.set(false);
    rightLed.set(false);
  }

  public void setLight(boolean isLightOn) {
    leftLed.set(isLightOn);
    rightLed.set(isLightOn);
  }

  public void setLightBlink(boolean isFloorControl, int blinkFrequency) {
    this.blinkingFrequency = blinkFrequency; 
    if (isInBlink) {
      return;
    }

    isInBlink = true;

    if (!isFloorControl) {
      blinkingFrequency = 6;
    }

    Thread thread = new Thread(() -> {
      if (coralShooterSubsystem.isGetTarget()) {
        for (int i = 0; i < blinkingFrequency; i++) {
          leftLed.set(true);
          rightLed.set(true);
          try {
            Thread.sleep(100);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
          leftLed.set(false);
          rightLed.set(false);
          try {
            Thread.sleep(100);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        }
        isInBlink = false;
      }
    });
    thread.setDaemon(true); // Set the thread as a daemon thread
    thread.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!isInBlink) {
      setLight(coralShooterSubsystem.isGetTarget());
    }
  }

  public Command setLightBlinkCmd(boolean isFloorControl, int blinkFrequency) {
    Command cmd = runOnce(() -> setLightBlink(isFloorControl, blinkFrequency));
    cmd.setName("setLightBlinkCmd");
    return cmd;
  }
}
