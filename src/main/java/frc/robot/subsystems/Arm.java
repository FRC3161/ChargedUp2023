package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.PIDConstants;
import frc.lib.util.ProfiledPIDConstants;
import frc.robot.Constants;
import frc.robot.Robot;

public class Arm extends ProfiledPIDSubsystem {

  // Motors
  private final CANSparkMax armLeader = new CANSparkMax(Constants.Arm.leaderMotorID, MotorType.kBrushless);
  private final CANSparkMax armFollower = new CANSparkMax(Constants.Arm.followerMotorID, MotorType.kBrushless);

  // Encoders
  private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(Constants.Arm.encoderDIOPort);

  // Motion Control
  private PIDController armRotationPID = new PIDController(Constants.Arm.armPID[0], Constants.Arm.armPID[1],
      Constants.Arm.armPID[2]);

  public Arm() {

    super(
        new ProfiledPIDController(Constants.Arm.armPID[0], Constants.Arm.armPID[1],
            Constants.Arm.armPID[2], Constants.Arm.armConstraints));
    enable();
    addChild("Arm PID", m_controller);

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
    setGoal(this.getEncoderPositionWithOffset());

    // PID
    SmartDashboard.putNumber("Arm rotation setpoint", 0);
    SmartDashboard.putNumber("Arm rotation encoder", 0);
    SmartDashboard.putString("arm limit", "none");
  }

  public double getEncoderPosition() {
    return Units.degreesToRadians(this.absoluteEncoder.getDistance() * (360 / 4));
  }

  public double getEncoderPositionWithOffset() {
    if (Robot.isSimulation()) {
      return this.m_controller.getSetpoint().position;
    }

    double encoderValue = this.getEncoderPosition() - Constants.Arm.encoderOffset;

    // hopefully fixes rollover issue
    return ((encoderValue + Math.PI) % (2 * Math.PI)) - Math.PI;
  }

  /**
   * @param angle in degrees PLS
   */
  public void setArmSetpoint(double angle) {
    angle = Units.degreesToRadians(angle);
    if (angle > Constants.Arm.maxAngle) {
      setGoal(Constants.Arm.maxAngle);
    } else if (angle < Constants.Arm.minAngle) {
      setGoal(Constants.Arm.minAngle);
    } else {
      setGoal(angle);
    }
  }

  /**
   * @param angle in degrees
   */
  public void overrideMotion(double angle) {
    angle = Units.degreesToRadians(angle);
    if (angle > Constants.Arm.maxAngle) {
      angle = Constants.Arm.maxAngle;
    } else if (angle < Constants.Arm.minAngle) {
      angle = Constants.Arm.minAngle;
    }

    setGoal(angle);
  }

  public boolean atSetpoint() {
    return this.m_controller.atGoal();
  }

  @Override
  public double getMeasurement() {
    return this.getEncoderPositionWithOffset();
  }

  public double handleFF(State setpoint) {
    return Constants.Arm.armFF.calculate(setpoint.position,
        setpoint.velocity);
  }

  @Override
  protected void useOutput(double output, State setpoint) {
    double ffOutput = this.handleFF(setpoint);

    this.armLeader.set(output + ffOutput);

    SmartDashboard.putNumber("Arm PID Output", output);
    SmartDashboard.putNumber("Arm FF Output", ffOutput);
    SmartDashboard.putNumber("Arm Total Output", ffOutput + output);
  }

}
