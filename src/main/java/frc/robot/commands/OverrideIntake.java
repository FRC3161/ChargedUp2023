package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Wrist;

public class OverrideIntake extends CommandBase {
  private final Wrist wrist;
  private final Arm arm;
  private LEDs leds;

  public OverrideIntake(Arm arm, Wrist wrist, LEDs leds) {
    this.arm = arm;
    this.wrist = wrist;
    this.leds = leds;
  }

  @Override
  public void initialize() {
    this.wrist.intakeIn(this.wrist.currentPiece, 0.2);
  }

  @Override
  public void end(boolean interrupted) {
    this.wrist.intakeStop();
  }
}
