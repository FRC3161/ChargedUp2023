package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.PIDConstants;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.GamePieceLevel;
import frc.robot.Constants.PieceType;

public class Wrist extends ProfiledPIDSubsystem {

  // Motors
  private final CANSparkMax wristMotor = new CANSparkMax(Constants.Wrist.wristMotorID, MotorType.kBrushless);
  public final TalonFX intakeMotor = new TalonFX(Constants.Wrist.intakeMotorID);

  // Encoder
  private final RelativeEncoder wristEncoder;
  private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(
      Constants.Wrist.absoluteEncoderPort);
  private double previousAbsoluteEncoder = 0.5;
  private double gooseEncoderRollover = 0.0;

  public PieceType currentPiece = PieceType.AIR;
  public GamePieceLevel gamePieceLevel = GamePieceLevel.L1;

  public double gooseEncoderValue = 0;

  // Beam break
  private DigitalInput breambreak = new DigitalInput(Constants.Wrist.beambreakDIO);

  public Wrist() {

    super(new ProfiledPIDController(Constants.Wrist.wristRotationPID[0], Constants.Wrist.wristRotationPID[1],
        Constants.Wrist.wristRotationPID[2], Constants.Wrist.constraints));
    enable();
    addChild("Wrist PID", m_controller);

    this.intakeMotor.configVoltageCompSaturation(Constants.Swerve.voltageComp);
    this.intakeMotor.enableVoltageCompensation(true);

    this.wristMotor.restoreFactoryDefaults();
    this.wristMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    this.wristMotor.setSmartCurrentLimit(30);
    this.wristMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // Encoder
    this.wristEncoder = this.wristMotor.getEncoder();
    this.wristEncoder.setPositionConversionFactor(360 / Constants.Wrist.wristGearRatio);

    Timer.delay(2);
    double encoderAbsoluteValue = this.absoluteEncoder.getAbsolutePosition();
    // if (encoderAbsoluteValue > 0 && encoderAbsoluteValue < 0.5) {
    // this.previousAbsoluteEncoder = encoderAbsoluteValue - 0.01;
    // this.gooseEncoderRollover = 1;
    // }
    this.getGooseEncoder();
    this.syncEncoders();
    m_controller.setIntegratorRange(-0.06, 0.06);

    TalonFXConfiguration intakeMotorConfiguration = new TalonFXConfiguration();
    // this.wristRotationPidConstants.sendDashboard("Wrist Rotation");
    // this.intakePIDConstants.sendDashboard("intake pid");
    // SmartDashboard.putNumber("set velocity", intakeSetVelocity);
    // intakeMotorConfiguration.supplyCurrLimit = new
    // SupplyCurrentLimitConfiguration(
    // true,
    // 20,
    // 20,
    // 0.1);

    intakeMotor.configFactoryDefault();
    intakeMotor.configAllSettings(intakeMotorConfiguration);

    SmartDashboard.putString("wrist limit", "none");
  }

  public void intakeIn(PieceType gamePiece) {
    if (gamePiece == PieceType.CONE) {
      this.intakeMotor.set(ControlMode.PercentOutput, Constants.Wrist.intakeInCone);
    } else if (gamePiece == PieceType.CUBE) {
      this.intakeMotor.set(ControlMode.PercentOutput, Constants.Wrist.intakeInCube);
    }
  }

  public void intakeIn(PieceType gamePiece, double power) {
    if (gamePiece == PieceType.CONE) {
      this.intakeMotor.set(ControlMode.PercentOutput, -power);
    } else if (gamePiece == PieceType.CUBE) {
      this.intakeMotor.set(ControlMode.PercentOutput, power);
    }
  }

  public void intakeOut(PieceType gamePiece) {
    if (gamePiece == PieceType.CONE) {
      switch (this.gamePieceLevel) {
        case L1:
          this.intakeMotor.set(ControlMode.PercentOutput, Constants.Wrist.outakeConeL1);
          break;
        case L2:
          this.intakeMotor.set(ControlMode.PercentOutput, Constants.Wrist.outakeConeL2);
          break;
        case L3:
          this.intakeMotor.set(ControlMode.PercentOutput, Constants.Wrist.outakeConeL3);
          break;

        default:
          break;
      }
    } else if (gamePiece == PieceType.CUBE) {
      switch (this.gamePieceLevel) {
        case L1:
          this.intakeMotor.set(ControlMode.PercentOutput, Constants.Wrist.outakeCubeL2);
          break;
        case L2:
          this.intakeMotor.set(ControlMode.PercentOutput, Constants.Wrist.outakeCubeL2);
          break;
        case L3:
          this.intakeMotor.set(ControlMode.PercentOutput, Constants.Wrist.outakeCubeL3);
          break;

        default:
          break;
      }
    }
  }

  public void intakeStop() {
    this.intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  public boolean getBeambreak() {
    return !this.breambreak.get();
  }

  public PieceType getGamePieceType() {
    return currentPiece;
  }

  /**
   * 
   * @param angle pls in radians
   */
  public void setWristSetpoint(double angle) {
    if (angle > Constants.Wrist.maxAngle) {
      setGoal(Constants.Wrist.maxAngle);
    } else if (angle < Constants.Wrist.minAngle) {
      setGoal(Constants.Wrist.minAngle);
    } else {
      setGoal(angle);
    }
  }

  /**
   * @param angle in radians
   */
  public void overrideMotion(double angle) {
    if (angle > Constants.Wrist.maxAngle) {
      angle = Constants.Wrist.maxAngle;
    } else if (angle < Constants.Wrist.minAngle) {
      angle = Constants.Wrist.minAngle;
    }

    setGoal(angle);
  }

  public boolean atSetpoint() {
    return m_controller.atGoal();
  }

  public double getWristSetpoint() {
    return m_controller.getSetpoint().position;
  }

  public double getEncoderPosition() {
    return Units.degreesToRadians(this.wristEncoder.getPosition());
  }

  public double getAbsoluteEncoder() {
    if (Robot.isSimulation()) {
      return getWristSetpoint();
    }

    return Units.degreesToRadians(wristEncoder.getPosition());

    // double encoderValue = gooseEncoderValue - Constants.Wrist.positionOffset;
    // return encoderValue;

    // return encoderValue;
    // double encoderValue =
    // Units.degreesToRadians(this.absoluteEncoder.getDistance() * 360)
    // - Constants.Wrist.positionOffset;
    // return encoderValue;
    // hopefully fixes rollover issue

    // return encoderValue % Math.PI;

  }

  private double handleFF() {
    double power = 0;
    power = Constants.Wrist.wristRotationFF.calculate(m_controller.getSetpoint().position,
        m_controller.getSetpoint().velocity);

    return power;
  }

  public double getGooseEncoder() {
    double enc = this.absoluteEncoder.getAbsolutePosition();
    // if ((this.previousAbsoluteEncoder < 0.25) && (enc > 0.75)) {
    // this.gooseEncoderRollover = this.gooseEncoderRollover - 1;
    // } else if ((this.previousAbsoluteEncoder > 0.75) && (enc < 0.25)) {
    // this.gooseEncoderRollover = this.gooseEncoderRollover + 1;
    // }
    // this.gooseEncoderValue = Units.degreesToRadians((enc +
    // this.gooseEncoderRollover) * 360);
    // return Units.degreesToRadians((enc + this.gooseEncoderRollover) * 360);
    this.gooseEncoderValue = Units.degreesToRadians(enc * 360);
    return Units.degreesToRadians(enc * 360);
    // TODO: add offset / - Constants.Wrist.absolutePositionOffset

  }

  public void syncEncoders() {
    // double absoluteEncoder = this.getAbsoluteEncoder();
    // if (absoluteEncoder < 0) {
    // this.wristEncoder.setPosition(Units.radiansToDegrees(3.114573));
    // this.wristSetPoint = 3.114573;
    // } else {
    // this.wristEncoder.setPosition(Units.radiansToDegrees(this.getAbsoluteEncoder()));
    // this.wristSetPoint = this.getAbsoluteEncoder();
    // }
    // double absoluteEncoder = Units
    // .degreesToRadians((this.absoluteEncoder.getAbsolutePosition() +
    // this.gooseEncoderRollover) * 360);
    // this.wristEncoder.setPosition(Units.radiansToDegrees(getAbsoluteEncoder()));
    // this.wristSetPoint = getAbsoluteEncoder();
    this.wristEncoder.setPosition(Units.radiansToDegrees(3.0573));
    this.setWristSetpoint(3.0573);
  }

  @Override
  protected double getMeasurement() {
    return this.getAbsoluteEncoder();
  }

  @Override
  protected void useOutput(double output, State setpoint) {
    double ffOutput = handleFF();

    this.wristMotor.set(ffOutput + output);

    SmartDashboard.putNumber("Wrist PID Output", output);
    SmartDashboard.putNumber("Wrist FF Output", ffOutput);
    SmartDashboard.putNumber("Wrist Total Output", ffOutput + output);
  }

  @Override
  public void periodic() {
    super.periodic();
    this.getGooseEncoder();
    this.previousAbsoluteEncoder = this.absoluteEncoder.getAbsolutePosition();
  }
}
