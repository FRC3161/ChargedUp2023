package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.presets.ConeL3Score;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Wrist;

public class DefaultAuto {
  private Arm arm;
  private Wrist wrist;
  private LEDs leds;

  public DefaultAuto(Arm arm, Wrist wrist, LEDs leds) {
    this.arm = arm;
    this.wrist = wrist;
    this.leds = leds;
  }

  public CommandBase getCommand() {
    return new ConeL3Score(arm, wrist, leds);
  }
}
