package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.PieceType;
import frc.robot.commands.presets.Rest;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj.Timer;

public class IntakeIn extends CommandBase {
  private final Wrist wrist;
  private final Arm arm;
  private PieceType gamePieceType;
  private Timer stopWatch;
  private LEDs leds;
  private boolean hasSeen = false;
  private boolean delayedCommand = false;
  private boolean override = false;

  public IntakeIn(Arm arm, Wrist wrist, PieceType gamePiece, LEDs leds) {
    this.arm = arm;
    this.wrist = wrist;
    this.gamePieceType = gamePiece;
    this.leds = leds;
  }

  public IntakeIn(Arm arm, Wrist wrist, PieceType gamePiece, LEDs leds, boolean delay) {
    this.arm = arm;
    this.wrist = wrist;
    this.gamePieceType = gamePiece;
    this.leds = leds;
    this.delayedCommand = delay;
  }

  public IntakeIn(Arm arm, Wrist wrist, LEDs leds) {
    this.arm = arm;
    this.wrist = wrist;
    this.leds = leds;

    this.override = true;
    this.gamePieceType = this.wrist.currentPiece;
  }

  @Override
  public void initialize() {
    this.wrist.intakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 30, 0.1));
    this.stopWatch = new Timer();
    if (this.wrist.getBeambreak()) {
      this.end(false);
    } else {
      this.wrist.currentPiece = this.gamePieceType;
      this.wrist.intakeIn(this.gamePieceType);
      this.hasSeen = false;
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
    if (this.override) {
      this.leds.set(Constants.LEDConstants.redOrange);
      return false;
    }

    this.leds.set(Constants.LEDConstants.solidRed);
    if (this.hasSeen) {

      if (this.delayedCommand) { // Use delay for Cube HP
        //
        if (this.stopWatch.hasElapsed(0.2)) {
          this.leds.set(Constants.LEDConstants.solidGreen);
          this.wrist.intakeStop();
        }

        if (this.stopWatch.hasElapsed(Constants.Wrist.CubeHPDelay / 1000)) {

          Rest.forceSet(arm, wrist);
          return true;

        }
        return false;
        //
      } else {
        if (this.stopWatch.hasElapsed(.2)) {
          this.leds.set(Constants.LEDConstants.solidGreen);
          Rest.forceSet(arm, wrist);
          return true;
        }
        return false;
      }
    }

    if (this.wrist.getBeambreak()) {
      this.hasSeen = true;
      this.stopWatch.start();
      return false;
    }
    return false;
  }

}
