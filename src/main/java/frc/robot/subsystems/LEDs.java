package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {
  private final Spark blinkin = new Spark(Constants.LEDConstants.blinkinPort);
  private double currentColor = 0;

  public void set(double value) {
    if (currentColor == value && this.blinkin.get() == value)
      return;

    if (value >= -1.0 && value <= 1.0) {
      currentColor = value;
      this.blinkin.set(value);
    }
  }
}