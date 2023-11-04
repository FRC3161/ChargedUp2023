package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Swerve;

public class SnakeSwerve extends TeleopSwerve {

  public SnakeSwerve(
      Swerve swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      BooleanSupplier robotCentricSup,
      BooleanSupplier rightBumper,
      BooleanSupplier NOSMode,
      BooleanSupplier faceForward,
      BooleanSupplier faceRight,
      BooleanSupplier faceBackwards,
      BooleanSupplier faceLeft) {
    super(swerve, translationSup, strafeSup, (DoubleSupplier) () -> 0, robotCentricSup, NOSMode,
        faceForward, faceRight,
        faceBackwards, faceLeft);
    this.defenseOverride = true;
  }

  @Override
  public double pointTo() {
    double angle = super.pointTo();
    if (angle != -1)
      return angle;

    double[] joystickValues = this.getJoystickValues();
    double translationVal = joystickValues[0];
    double strafeVal = joystickValues[1];

    double rotationDegree = 0;

    SmartDashboard.putNumber("translationVal", translationVal);
    SmartDashboard.putNumber("strafeVal", strafeVal);

    if (strafeVal != 0 && translationVal != 0) {
      rotationDegree = Math.atan(-strafeVal / translationVal) * (180 / Math.PI);
      if (translationVal < 0 && strafeVal < 0) {
        rotationDegree *= -1;
        rotationDegree += 90;
      } else if (translationVal < 0 && strafeVal > 0) {
        rotationDegree += 180;
      } else if (translationVal > 0 && strafeVal > 0) {
        rotationDegree *= -1;
        rotationDegree += 270;
      }
    } else if (strafeVal == 0 && translationVal == 0) {
      rotationDegree = -1;
    } else if (strafeVal == 0) {
      rotationDegree = translationVal > 0 ? 0 : 180;
    } else {
      rotationDegree = strafeVal > 0 ? 270 : 90;
    }

    if (rotationDegree >= 0) {
      this.swerve.setHold(rotationDegree);
    }
    return rotationDegree;
  }

}
