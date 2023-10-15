package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.PIDConstants;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.GamePieceLevel;
import frc.robot.Constants.PieceType;

public class Wrist extends SubsystemBase {

  // Motion profile
  private TrapezoidProfile m_motionProfile = new TrapezoidProfile(
      Constants.Arm.armConstraints, new State(0, 0));
  private State m_setpoint = new State();
  private double m_currentPosition = 0;
  private State m_goal = new State();
  private Timer m_motionTimer = new Timer();

  // Motors
  private final CANSparkMax wristMotor = new CANSparkMax(Constants.Wrist.wristMotorID, MotorType.kBrushless);
  public final TalonFX intakeMotor = new TalonFX(Constants.Wrist.intakeMotorID);

  // Encoder
  private final RelativeEncoder wristEncoder;
  private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(
      Constants.Wrist.absoluteEncoderPort);
  private double previousAbsoluteEncoder = 0.5;
  private double gooseEncoderRollover = 0.0;

  // PID
  private final PIDConstants wristRotationPidConstants = Constants.Wrist.wristRotationPID;
  private final PIDController wristRotationPID = Constants.Wrist.wristRotationPID.getController();
  private final PIDConstants intakePIDConstants = Constants.Wrist.intakePIDConstants;
  private final PIDController intakePIDController = Constants.Wrist.intakePIDConstants.getController();
  private double intakeSetVelocity = 0;
  public boolean isCorrecting = false;

  // Color sensor
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  public final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  public PieceType currentPiece = PieceType.AIR;
  public GamePieceLevel gamePieceLevel = GamePieceLevel.L1;
  public int colorCounter = 0;

  public double gooseEncoderValue = 0;

  // Beam break
  private DigitalInput breambreak = new DigitalInput(Constants.Wrist.beambreakDIO);

  public Wrist() {
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
    this.wristRotationPID.setIntegratorRange(-0.06, 0.06);

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
    isCorrecting = false;
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
    if (colorSensor.isConnected()) {
      Color detectedColor = colorSensor.getColor();
      PieceType output;

      int proximity = colorSensor.getProximity();
      if (proximity < 55) {
        output = PieceType.AIR;
      } else {
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("Proximity", proximity);
        if (detectedColor.red > 0.17 && detectedColor.red < 0.33 && detectedColor.green > 0.27
            && detectedColor.green < 0.48
            && detectedColor.blue < 0.49 && detectedColor.blue > 0.27) {
          output = PieceType.CUBE;
        } else if (detectedColor.red > 0.31 && detectedColor.red < 0.40 && detectedColor.green > 0.45
            && detectedColor.green < 0.55 && detectedColor.blue > 0 && detectedColor.blue < 0.23) {
          output = PieceType.CONE;
        } else {
          output = PieceType.AIR;
        }
      }
      return output;
    } else {
      return this.currentPiece;
    }

  }

  private void updateTrapezoidProfile(double angle) {
    m_goal = new State(angle, 0);
    m_motionProfile = new TrapezoidProfile(Constants.Wrist.constraints, m_goal,
        new State(m_currentPosition, m_setpoint.velocity));
    m_motionTimer = new Timer();
    m_motionTimer.start();

  }

  /**
   * 
   * @param angle pls in radians
   */
  public void setWristSetpoint(double angle) {
    if (angle > Constants.Wrist.maxAngle) {
      this.updateTrapezoidProfile(Constants.Wrist.maxAngle);
    } else if (angle < Constants.Wrist.minAngle) {
      this.updateTrapezoidProfile(Constants.Wrist.minAngle);
    } else {
      this.updateTrapezoidProfile(angle);
    }
  }

  /**
   * @param angle in radians
   */
  public void overrideMotion(double angle) {
    if (angle > Constants.Wrist.maxAngle) {
      angle = Constants.Wrist.maxAngle;
    } else if (angle < Constants.Wrist.minAngle) {
      this.updateTrapezoidProfile(Constants.Wrist.minAngle);
      angle = Constants.Wrist.minAngle;
    }

    m_goal = new State(angle, 0);
    m_motionProfile = new TrapezoidProfile(Constants.Wrist.constraints, m_goal, m_goal);
    m_motionTimer = new Timer();
    m_motionTimer.start();

  }

  public boolean atSetpoint() {
    return m_currentPosition > m_goal.position + Units.degreesToRadians(-20)
        && m_currentPosition < m_goal.position + Units.degreesToRadians(20);
  }

  public double getWristSetpoint() {
    return m_setpoint.position;
  }

  public double getEncoderPosition() {
    return Units.degreesToRadians(this.wristEncoder.getPosition());
  }

  public double getAbsoluteEncoder() {
    if (Robot.isSimulation()) {
      return m_setpoint.position;
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

  public double handleMovement() {
    double power = 0;
    power = this.wristRotationPID.calculate(m_currentPosition,
        m_setpoint.position);
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
  public void periodic() {
    this.getGooseEncoder();
    m_currentPosition = this.getAbsoluteEncoder();

    m_setpoint = m_motionProfile.calculate(m_motionTimer.get());

    double power = MathUtil.clamp(this.handleMovement(), -1, 1);
    // if (Math.abs(this.getAbsoluteEncoder()) > 4) { // reading is out of range
    // this.wristMotor.set(0);
    // this.wristMotor.disable();
    // } else {
    // }

    this.wristMotor.set(power);

    // intakeSetVelocity = SmartDashboard.getNumber("set velocity",
    // intakeSetVelocity);
    // double intakePower =
    // intakePIDController.calculate(intakeMotor.getSelectedSensorVelocity(),
    // intakeSetVelocity);
    // SmartDashboard.putNumber("intake velocity",
    // intakeMotor.getSelectedSensorVelocity());
    // SmartDashboard.putNumber("intake power", intakePower);

    // this.intakeMotor.set(ControlMode.PercentOutput, intakePower);

    SmartDashboard.putNumber("Wrist Goal", this.m_goal.position);
    SmartDashboard.putNumber("Wrist setpoint", this.m_setpoint.position);
    SmartDashboard.putNumber("Wrist current position", this.m_currentPosition);

    // SmartDashboard.putNumber("Wrist setpoint", this.m_setpoint.position);
    // SmartDashboard.putNumber("Wrist relative encoder",
    // this.getEncoderPosition());
    // SmartDashboard.putNumber("Wrist pid output", power);
    // SmartDashboard.putBoolean("wrist end", this.atSetpoint());
    // SmartDashboard.putBoolean("beambreak", this.breambreak.get());
    // SmartDashboard.putNumber("wrist absolute error", this.getAbsoluteEncoder() -
    // this.m_setpoint.position);
    // SmartDashboard.putNumber("Wrist absolute encoder",
    // this.getAbsoluteEncoder());
    // SmartDashboard.putNumber("raw value absolute",
    // this.absoluteEncoder.getAbsolutePosition());
    // SmartDashboard.putNumber("goose absolute encoder", this.gooseEncoderValue);
    // SmartDashboard.putNumber("goose rollover", this.gooseEncoderRollover);

    this.previousAbsoluteEncoder = this.absoluteEncoder.getAbsolutePosition();

  }
}
