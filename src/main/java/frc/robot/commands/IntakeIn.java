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
  private boolean auto = false;
  private Timer stopWatch;
  private LEDs leds;
  private boolean hasSeen = false;
  private boolean delayedCommand = false;

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
      this.stopWatch = new Timer();
      this.stopWatch.start();
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
  
      if (this.delayedCommand) { // Use delay for Cube HP
        //
        if (this.stopWatch.hasElapsed(0.2)){
          this.leds.set(Constants.LEDConstants.solidGreen);
          this.wrist.intakeStop();
        }
  
        if (this.stopWatch.hasElapsed(Constants.Wrist.CubeHPDelay/1000)) {
          
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
