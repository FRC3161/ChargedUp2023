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
import frc.robot.Constants.GamePieceLevel;
import frc.robot.Constants.PieceType;

public class Wrist extends SubsystemBase {

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
  private double wristSetPoint = 0;
  private double intakeSetVelocity = 0;

  // Color sensor
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  public final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  public PieceType currentPiece = PieceType.AIR;
  public GamePieceLevel gamePieceLevel = GamePieceLevel.L1;
  public int colorCounter = 0;

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
    this.syncEncoders();

    TalonFXConfiguration intakeMotorConfiguration = new TalonFXConfiguration();
    this.wristRotationPidConstants.sendDashboard("Wrist Rotation");
    this.intakePIDConstants.sendDashboard("intake pid");
    SmartDashboard.putNumber("set velocity", intakeSetVelocity);
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

  /**
   * 
   * @param angle pls in radians
   */
  public void setWristSetpoint(double angle) {
    this.wristSetPoint = angle;
    if (this.wristSetPoint > Constants.Wrist.maxAngle) {
      this.wristSetPoint = Constants.Wrist.maxAngle;
    } else if (wristSetPoint < Constants.Wrist.minAngle) {
      this.wristSetPoint = Constants.Wrist.minAngle;
    }
    this.handleMovement();
  }

  public boolean atSetpoint() {
    return this.getEncoderPosition() > this.wristSetPoint + Units.degreesToRadians(-5)
        && this.getEncoderPosition() < this.wristSetPoint + Units.degreesToRadians(5);
  }

  public double getWristSetpoint() {
    return this.wristSetPoint;
  }

  public double getEncoderPosition() {
    return Units.degreesToRadians(this.wristEncoder.getPosition());
  }

  public double getAbsoluteEncoder() {
    double encoderValue = Units.degreesToRadians(this.absoluteEncoder.getDistance() * 360)
        - Constants.Wrist.positionOffset;
    return encoderValue;
    // hopefully fixes rollover issue
    // return ((encoderValue + Math.PI) % (2 * Math.PI)) - Math.PI;
  }

  public double handleMovement() {
    this.wristRotationPidConstants.retrieveDashboard(this.wristRotationPID);
    if (this.wristSetPoint > Constants.Wrist.maxAngle) {
      this.wristSetPoint = Constants.Wrist.maxAngle;
    } else if (wristSetPoint < Constants.Wrist.minAngle) {
      this.wristSetPoint = Constants.Wrist.minAngle;
    }
    double power = 0;
    power = this.wristRotationPID.calculate(this.getEncoderPosition(),
        this.wristSetPoint);

    return power;
  }

  public double getGooseEncoder() {
    double enc = this.absoluteEncoder.getAbsolutePosition();
    if ((this.previousAbsoluteEncoder < 0.25) && (enc > 0.75)) {
      this.gooseEncoderRollover = this.gooseEncoderRollover - 1;
    } else if ((this.previousAbsoluteEncoder > 0.75) && (enc < 0.25)) {
      this.gooseEncoderRollover = this.gooseEncoderRollover + 1;
    }
    return Units.degreesToRadians((enc + this.gooseEncoderRollover) * 360);
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
    // double absoluteEncoder = getAbsoluteEncoder();
    // this.wristEncoder.setPosition(Units.radiansToDegrees(absoluteEncoder));
    // this.wristSetPoint = absoluteEncoder;
    this.wristEncoder.setPosition(Units.radiansToDegrees(3.114573));
    this.wristSetPoint = 3.114573;
  }

  @Override
  public void periodic() {
    wristRotationPidConstants.retrieveDashboard(intakePIDController);

    double power = MathUtil.clamp(this.handleMovement(), -0.7, 0.7);
    if (Math.abs(this.getAbsoluteEncoder()) > 4) { // reading is out of range
      this.wristMotor.set(0);
      this.wristMotor.disable();
    } else {
      this.wristMotor.set(power);
    }
    // intakeSetVelocity = SmartDashboard.getNumber("set velocity",
    // intakeSetVelocity);
    // double intakePower =
    // intakePIDController.calculate(intakeMotor.getSelectedSensorVelocity(),
    // intakeSetVelocity);
    // SmartDashboard.putNumber("intake velocity",
    // intakeMotor.getSelectedSensorVelocity());
    // SmartDashboard.putNumber("intake power", intakePower);

    // this.intakeMotor.set(ControlMode.PercentOutput, intakePower);
    SmartDashboard.putNumber("Wrist setpoint", this.wristSetPoint);
    SmartDashboard.putNumber("Wrist relative encoder", this.getEncoderPosition());
    SmartDashboard.putNumber("Wrist pid output", power);
    SmartDashboard.putBoolean("wrist end", this.atSetpoint());
    SmartDashboard.putBoolean("beambreak", this.breambreak.get());
    SmartDashboard.putNumber("wrist absolute error", this.getAbsoluteEncoder() - this.wristSetPoint);
    SmartDashboard.putNumber("Wrist absolute encoder", this.getAbsoluteEncoder());
    SmartDashboard.putNumber("raw value absolute", this.absoluteEncoder.get());
    SmartDashboard.putNumber("goose absolute encoder", Units.radiansToDegrees(this.getGooseEncoder()));

    this.previousAbsoluteEncoder = this.absoluteEncoder.getAbsolutePosition();

  }
}
