package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Swerve;

public class Balance extends CommandBase {
  private final Swerve swerve;
  private PIDController balanceController = Constants.Swerve.balancePID.getController();
  private final LEDs leds;

  public Balance(Swerve swerve, LEDs leds) {
    this.swerve = swerve;
    this.leds = leds;
    this.addRequirements(swerve);
  }

  @Override
  public void execute() {
    double power = 0;
    power = this.balanceController.calculate(this.swerve.getPitch().getDegrees(), -3);

    this.swerve.drive(new Translation2d(power, 0), 0, true, true, true, true);

    if (isBalanced()) {
      leds.set(Constants.LEDConstants.raindbow);
    } else {
      leds.set(Constants.LEDConstants.skyblue);
    }
  }

  public boolean isBalanced() {
    double value = this.swerve.getPitch().getDegrees();
    return value - 3 <= Constants.Swerve.balancePID.tolerance && value + 3 >= -Constants.Swerve.balancePID.tolerance;
  }

  @Override
  public void end(boolean interrupted) {
    this.leds.set(Constants.LEDConstants.off);
  }
}