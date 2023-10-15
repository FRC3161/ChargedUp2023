package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class ArmManualControl extends CommandBase {
  private Arm arm;
  private Wrist wrist;
  private DoubleSupplier armYSupplier;
  private DoubleSupplier wristYSupplier;
  private double maximumDisplacement = 40; // 40 degrees, might need to increase this if it's too slow

  public ArmManualControl(Arm arm, Wrist wrist, DoubleSupplier armYSupplier, DoubleSupplier wristYSupplier) {
    this.arm = arm;
    this.wrist = wrist;
    this.armYSupplier = armYSupplier;
    this.wristYSupplier = wristYSupplier;

    addRequirements(arm, wrist);
  }

  @Override
  public void execute() {
    // Arm
    double armY = -MathUtil.applyDeadband(this.armYSupplier.getAsDouble(), Constants.Swerve.stickDeadband);
    double armChange = (armY * maximumDisplacement) / 50;
    // HAVE TO FIX THIS
    if (armY != 0)
      this.arm.overrideMotion(Units.radiansToDegrees(this.arm.getEncoderPositionWithOffset()) + armChange);

    // Wrist
    double wristY = -MathUtil.applyDeadband(this.wristYSupplier.getAsDouble(), Constants.Swerve.stickDeadband);
    double wristChange = (wristY * 80) / 50;

    if (wristY != 0)
      this.wrist.overrideMotion(this.wrist.getWristSetpoint() + Units.degreesToRadians(wristChange));
  }
}
