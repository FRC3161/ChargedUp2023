package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotModes;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends CommandBase {
  public final Swerve swerve;
  private final DoubleSupplier translationSup;
  public final DoubleSupplier strafeSup;
  public DoubleSupplier rotationSup;
  private final BooleanSupplier robotCentricSup;
  private final BooleanSupplier NOSMode;
  private final BooleanSupplier faceForward;
  private final BooleanSupplier faceRight;
  private final BooleanSupplier faceBackwards;
  private final BooleanSupplier faceLeft;

  public boolean defenseOverride = false;

  private boolean rotationButtonsPressed;
  private boolean isDefense;

  /**
   * The default command for Swerve and which is being used for driving, if any
   * other command overrides this one,
   * all the driving abilities will be suspended until it's back on the command
   * scheduler
   * 
   * @param swerve          An instance of the swerve subsustem
   * @param translationSup  the value of translation axis on the controller
   * @param strafeSup       the value of starfe axis on the controller
   * @param rotationSup     the rotation value
   * @param robotCentricSup whether or not the robot is driving relative to the
   *                        field or relative to itself
   * @param NOSMode         If true, the power will increase from 60% to 100%
   * @param faceForward     Is the faceForward button pressed?
   * @param faceRight       Is the faceRight button pressed?
   * @param faceBackwards   Is the faceBackwards button pressed?
   * @param faceLeft        Is the faceLeft button pressed?
   */
  public TeleopSwerve(
      Swerve swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier robotCentricSup,
      BooleanSupplier NOSMode,
      BooleanSupplier faceForward,
      BooleanSupplier faceRight,
      BooleanSupplier faceBackwards,
      BooleanSupplier faceLeft) {
    this.swerve = swerve;
    addRequirements(swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
    this.NOSMode = NOSMode;
    this.faceForward = faceForward;
    this.faceRight = faceRight;
    this.faceBackwards = faceBackwards;
    this.faceLeft = faceLeft;

    this.rotationButtonsPressed = false;
    this.isDefense = true;
  }

  /**
   * A function to set the orientation hold of the robot if any of the face
   * buttons are pressed
   * 
   * @return -1 if no button is pressed, or the angle that the robot is going to
   *         point to if buttons are pressed.
   */
  public double pointTo() {
    boolean faceForward = this.faceForward.getAsBoolean();
    boolean faceRight = this.faceRight.getAsBoolean();
    boolean faceBackwards = this.faceBackwards.getAsBoolean();
    boolean faceLeft = this.faceLeft.getAsBoolean();

    int angle = -1;

    if (faceForward && faceRight) {
      angle = 45;
    } else if (faceRight && faceBackwards) {
      angle = 135;
    } else if (faceBackwards && faceLeft) {
      angle = 225;
    } else if (faceLeft && faceForward) {
      angle = 315;
    } else if (faceForward) {
      angle = 0;
    } else if (faceRight) {
      angle = 90;
    } else if (faceBackwards) {
      angle = 180;
    } else if (faceLeft) {
      angle = 270;
    } else if (this.swerve.robotRotationPID.atSetpoint()) {
      this.defenseOverride = false;
    }

    if (angle != -1 && !this.rotationButtonsPressed) {
      this.swerve.setHold(angle);
    }

    if (!faceForward && !faceRight && !faceBackwards && !faceLeft) {
      this.rotationButtonsPressed = false;
    } else {
      this.rotationButtonsPressed = true;
      this.defenseOverride = true;
    }

    return angle;
  }

  @Override
  public void initialize() {
    // Set the hold position to the current orientation of the robot
    this.swerve.resetHold();
  }

  public double[] getJoystickValues() {
    double translationVal, strafeVal, rotationVal;
    /* Get Values, Deadband */
    translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Swerve.stickDeadband);
    strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Swerve.stickDeadband);
    rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.Swerve.stickDeadband);

    double[] output = new double[] { translationVal, strafeVal, rotationVal };
    return output;
  }

  @Override
  public void execute() {
    double translationVal, strafeVal, rotationVal;
    double[] joystickValues = this.getJoystickValues();
    translationVal = joystickValues[0];
    strafeVal = joystickValues[1];
    rotationVal = joystickValues[2];

    this.pointTo();

    if (!this.NOSMode.getAsBoolean()) {
      if (robotCentricSup.getAsBoolean()) {
        translationVal *= 0.2;
        strafeVal *= 0.2;
        rotationVal *= 0.3;
      } else {
        translationVal *= 0.6;
        strafeVal *= 0.6;
        rotationVal *= 0.3;
      }
    }


    /* Drive */
    swerve.drive(
        new Translation2d(translationVal,
            strafeVal).times(Constants.Swerve.maxSpeed),
        rotationVal * Constants.Swerve.maxAngularVelocity,
        true,
        true, // True -> driving based on percent output, False -> driving based on PID and
              // FeedForward
        this.isDefense, this.defenseOverride);
  }
}
