package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.presets.Rest;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Wrist;

public class IntakeOut extends CommandBase {
  private final Arm arm;
  private final Wrist wrist;
  private final LEDs leds;
  private boolean beambreakInitial;
  private int counter = 0;

  public IntakeOut(Arm arm, Wrist wrist, LEDs leds) {
    this.arm = arm;
    this.wrist = wrist;
    this.leds = leds;
  }

  @Override
  public void initialize() {
    this.wrist.intakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration());
    this.wrist.intakeOut(this.wrist.currentPiece);
    this.beambreakInitial = this.wrist.getBeambreak();
    counter = 0;
  }

  @Override
  public void end(boolean interrupted) {
    if (beambreakInitial != this.wrist.getBeambreak()) {
      Rest.forceSet(arm, wrist);
    }
    this.wrist.intakeStop();
  }

  @Override
  public boolean isFinished() {
    if (!this.wrist.getBeambreak()) {
      if (counter > 10) {
        Rest.forceSet(arm, wrist);
        return true;
      }
      counter++;
      this.leds.set(Constants.LEDConstants.solidBlue);
      return false;
    }
    return false;
  }
}
