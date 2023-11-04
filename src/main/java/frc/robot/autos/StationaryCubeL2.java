package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.presets.CubeL2Score;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Wrist;

public class StationaryCubeL2 extends AutoBase {
  public StationaryCubeL2(Arm arm, Wrist wrist, LEDs leds) {
    this.arm = arm;
    this.wrist = wrist;
    this.leds = leds;
  }

  @Override
  public Pose2d getInitialHolonomicPose() {
    return new Pose2d();
  }

  @Override
  public Command getCommand() {
    return new CubeL2Score(arm, wrist, leds);
  }
}
