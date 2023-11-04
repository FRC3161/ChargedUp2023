package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.PIDConstants;
import frc.lib.util.ProfiledPIDConstants;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  // Motors
  private final CANSparkMax armLeader = new CANSparkMax(Constants.Arm.leaderMotorID, MotorType.kBrushless);
  private final CANSparkMax armFollower = new CANSparkMax(Constants.Arm.followerMotorID, MotorType.kBrushless);

  // Encoders
  private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(Constants.Arm.encoderDIOPort);

  // Motion Control
  private final PIDConstants armPidConstants = Constants.Arm.armPID;
  private PIDController armRotationPID = Constants.Arm.armPID.getController();

  public double armSetpoint = 0;

  public Arm() {
    /* Motors setup */

    // Arm leader
    this.armLeader.restoreFactoryDefaults();
    this.armLeader.enableVoltageCompensation(Constants.Swerve.voltageComp);
    this.armLeader.setSmartCurrentLimit(Constants.Arm.stallLimit, Constants.Arm.currentLimit);
    this.armLeader.setIdleMode(IdleMode.kBrake);

    // Arm follower
    this.armFollower.restoreFactoryDefaults();
    this.armFollower.enableVoltageCompensation(Constants.Swerve.voltageComp);
    this.armFollower.setSmartCurrentLimit(Constants.Arm.stallLimit, Constants.Arm.currentLimit);
    this.armFollower.follow(this.armLeader, true);
    this.armFollower.setIdleMode(IdleMode.kBrake);

    /* End Motors setup */

    // Encoder
    this.armSetpoint = this.getEncoderPositionWithOffset();

    // PID
    SmartDashboard.putNumber("Arm rotation setpoint", 0);
    SmartDashboard.putNumber("Arm rotation encoder", 0);
    SmartDashboard.putString("arm limit", "none");

    this.armPidConstants.sendDashboard("arm pid");
  }

  public double getEncoderPosition() {
    return Units.degreesToRadians(this.absoluteEncoder.getDistance() * (360 / 4));
  }

  public double getEncoderPositionWithOffset() {
    double encoderValue = this.getEncoderPosition() - Constants.Arm.encoderOffset;

    // hopefully fixes rollover issue
    return ((encoderValue + Math.PI) % (2 * Math.PI)) - Math.PI;
  }

  /**
   * @param angle in degrees PLS
   */
  public void setArmSetpoint(double angle) {
    this.armSetpoint = Units.degreesToRadians(angle);
    if (this.armSetpoint > Constants.Arm.maxAngle) {
      this.armSetpoint = Constants.Arm.maxAngle;
      SmartDashboard.putString("arm limit", "MAX EXCEEDED");
    } else if (this.armSetpoint < Constants.Arm.minAngle) {
      this.armSetpoint = Constants.Arm.minAngle;
      SmartDashboard.putString("arm limit", "MIN EXCEEDED");
    } else {
      SmartDashboard.putString("arm limit", "none");

    }
    this.handleMovement();
  }

  public boolean atSetpoint() {
    return this.getEncoderPositionWithOffset() > this.armSetpoint + Units.degreesToRadians(-5)
        && this.getEncoderPositionWithOffset() < this.armSetpoint + Units.degreesToRadians(5);
  }

  public double handleMovement() {
    double pidOutput;
    this.armPidConstants.retrieveDashboard(this.armRotationPID);
    pidOutput = MathUtil.clamp(this.armRotationPID
        .calculate(this.getEncoderPositionWithOffset(), this.armSetpoint), -0.05,
        Constants.Arm.armMaxOutput);

    return pidOutput;
  }

  @Override
  public void periodic() {
    double pidOutput = this.handleMovement();

    SmartDashboard.putNumber("Arm pid output", pidOutput);
    SmartDashboard.putNumber("Arm absolute encoder", this.getEncoderPosition());
    SmartDashboard.putNumber("Arm absolute encoder with offset",
        Units.radiansToDegrees(this.getEncoderPositionWithOffset()));
    SmartDashboard.putNumber("Arm rotation setpoint", Units.radiansToDegrees(this.armSetpoint));
    SmartDashboard.putNumber("error", Units.radiansToDegrees(this.getEncoderPositionWithOffset() - this.armSetpoint));
    SmartDashboard.putBoolean("arm end", this.atSetpoint());
    this.armLeader.set(pidOutput);
  }
}
