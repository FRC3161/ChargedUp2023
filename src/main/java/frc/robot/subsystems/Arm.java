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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.PIDConstants;
import frc.lib.util.ProfiledPIDConstants;
import frc.robot.Constants;
import frc.robot.Robot;

public class Arm extends SubsystemBase {
  // Motion control
  private TrapezoidProfile m_motionProfile = new TrapezoidProfile(
      Constants.Arm.armConstraints, new State(0, 0));
  private State m_setpoint = new State();
  private double m_currentPosition = 0;
  private State m_goal = new State();
  private Timer m_motionTimer = new Timer();

  // Motors
  private final CANSparkMax armLeader = new CANSparkMax(Constants.Arm.leaderMotorID, MotorType.kBrushless);
  private final CANSparkMax armFollower = new CANSparkMax(Constants.Arm.followerMotorID, MotorType.kBrushless);

  // Encoders
  private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(Constants.Arm.encoderDIOPort);

  // Motion Control
  private final PIDConstants armPidConstants = Constants.Arm.armPID;
  private PIDController armRotationPID = Constants.Arm.armPID.getController();

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
    this.m_currentPosition = this.getEncoderPositionWithOffset();
    updateTrapezoidProfile(new State(m_currentPosition, 0));

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
    if (Robot.isSimulation()) {
      return m_setpoint.position;
    }

    double encoderValue = this.getEncoderPosition() - Constants.Arm.encoderOffset;

    // hopefully fixes rollover issue
    return ((encoderValue + Math.PI) % (2 * Math.PI)) - Math.PI;
  }

  private void updateTrapezoidProfile(State goal) {
    m_goal = goal;
    m_motionProfile = new TrapezoidProfile(Constants.Arm.armConstraints, goal,
        new State(m_currentPosition, m_setpoint.velocity));
    m_motionTimer = new Timer();
    m_motionTimer.start();
  }

  /**
   * @param angle in degrees PLS
   */
  public void setArmSetpoint(double angle) {
    angle = Units.degreesToRadians(angle);
    if (angle > Constants.Arm.maxAngle) {
      updateTrapezoidProfile(new State(Constants.Arm.maxAngle, 0));
    } else if (angle < Constants.Arm.minAngle) {
      updateTrapezoidProfile(new State(Constants.Arm.minAngle, 0));
    } else {
      updateTrapezoidProfile(new State(angle, 0));
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

    m_goal = new State(angle, 0);
    m_motionProfile = new TrapezoidProfile(Constants.Arm.armConstraints, m_goal, m_goal);
    m_motionTimer = new Timer();
    m_motionTimer.start();
  }

  public boolean atSetpoint() {
    return m_currentPosition > m_goal.position + Units.degreesToRadians(-5)
        && m_currentPosition < m_goal.position + Units.degreesToRadians(5);
  }

  public double handleMovement() {
    double pidOutput;
    m_setpoint = m_motionProfile.calculate(m_motionTimer.get());
    // this.armPidConstants.retrieveDashboard(this.armRotationPID);

    pidOutput = MathUtil.clamp(this.armRotationPID
        .calculate(this.getEncoderPositionWithOffset(), m_setpoint.position), -1, 1);

    return pidOutput;
  }

  public double handleFF() {
    return Constants.Arm.armFF.calculate(m_setpoint.position, m_setpoint.velocity);
  }

  @Override
  public void periodic() {
    this.m_currentPosition = this.getEncoderPositionWithOffset();
    double pidOutput = this.handleMovement();
    double ffOutput = this.handleFF();
    this.armLeader.set(pidOutput);
    SmartDashboard.putNumber("Arm pid", pidOutput);
    SmartDashboard.putNumber("Arm ff", ffOutput);
    SmartDashboard.putNumber("Arm goal", m_goal.position);
    SmartDashboard.putNumber("Arm setpoint", m_setpoint.position);
    SmartDashboard.putNumber("Arm current position", m_currentPosition);

  }
}
