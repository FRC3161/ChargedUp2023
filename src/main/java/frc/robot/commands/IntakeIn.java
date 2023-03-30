package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.PieceType;
import frc.robot.commands.presets.Rest;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Wrist;

public class IntakeIn extends CommandBase {
  private final Wrist wrist;
  private final Arm arm;
  private PieceType gamePieceType;
  private boolean auto = false;
  private int counter = 0;
  private LEDs leds;
  private boolean hasSeen = false;

  public IntakeIn(Arm arm, Wrist wrist, PieceType gamePiece, LEDs leds) {
    this.arm = arm;
    this.wrist = wrist;
    this.gamePieceType = gamePiece;
    this.leds = leds;
  }

  public IntakeIn(Arm arm, Wrist wrist, PieceType gamePiece, boolean auto) {
    this.arm = arm;
    this.wrist = wrist;
    this.gamePieceType = gamePiece;
    this.auto = auto;
  }

  @Override
  public void initialize() {
    this.wrist.intakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 30, 0.1));
    if (this.wrist.getBeambreak()) {
      this.end(auto);
    } else {
      this.wrist.currentPiece = this.gamePieceType;
      this.wrist.intakeIn(this.gamePieceType);
      this.hasSeen = false;
      this.counter = 0;
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (!this.wrist.getBeambreak()) {
      this.leds.set(Constants.LEDConstants.off);
    }

    Rest.forceSet(arm, wrist);
    this.wrist.intakeStop();
  }

  @Override
  public boolean isFinished() {
    this.leds.set(Constants.LEDConstants.solidRed);
    if (this.hasSeen) {
      counter++;
      if (counter > 10) {
        this.leds.set(Constants.LEDConstants.solidGreen);
        Rest.forceSet(arm, wrist);
        return true;
      }
      return false;
    }

    if (this.wrist.getBeambreak()) {
      this.hasSeen = true;
      return false;
    }
    return false;
  }

}
