package frc.robot.subsystems;

import javax.swing.plaf.synth.ColorType;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3.RawColor;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.PIDConstants;
import frc.robot.Constants;
import frc.robot.Constants.PieceType;

public class Wrist extends SubsystemBase {

  // Motors
  private final CANSparkMax wristMotor = new CANSparkMax(Constants.Wrist.wristMotorID, MotorType.kBrushless);
  private final TalonFX intakeMotor = new TalonFX(Constants.Wrist.intakeMotorID);

  // Encoder
  private final RelativeEncoder wristEncoder;
  private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(Constants.Wrist.absoluteEncoderPort);

  // PID
  private final PIDConstants wristRotationPidConstants = Constants.Wrist.wristRotationPID;
  private final PIDController wristRotationPID = Constants.Wrist.wristRotationPID.getController();
  private double wristSetPoint = 0;

  // Color sensor
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  public final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  public PieceType currentPiece = PieceType.AIR;
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
    this.wristEncoder.setPosition(Units.radiansToDegrees(this.getAbsoluteEncoder()));

    this.wristSetPoint = this.getAbsoluteEncoder();

    TalonFXConfiguration intakeMotorConfiguration = new TalonFXConfiguration();
    this.wristRotationPidConstants.sendDashboard("Wrist Rotation");
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
      this.intakeMotor.set(ControlMode.PercentOutput, -0.6);
    } else if (gamePiece == PieceType.CUBE) {
      this.intakeMotor.set(ControlMode.PercentOutput, 0.5);
    }
  }

  public void intakeOut(PieceType gamePiece) {
    if (gamePiece == PieceType.CONE) {
      this.intakeMotor.set(ControlMode.PercentOutput, 7);
    } else if (gamePiece == PieceType.CUBE) {
      this.intakeMotor.set(ControlMode.PercentOutput, -1);
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
      SmartDashboard.putString("wrist limit", "MAX EXCEEDED");
    } else if (wristSetPoint < Constants.Wrist.minAngle) {
      this.wristSetPoint = Constants.Wrist.minAngle;
      SmartDashboard.putString("wrist limit", "MIN EXCEEDED");
    } else {
      SmartDashboard.putString("wrist limit", "none");
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

    // hopefully fixes rollover issue
    return ((encoderValue + Math.PI) % (2 * Math.PI)) - Math.PI;
  }

  public void resetWristEncoder() {
    this.wristEncoder.setPosition(0);
    this.wristSetPoint = 0;

    // CustomThreads.setTimeout(() -> {
    // this.wristSetPoint = 0;
    // }, 20);
  }

  public double handleMovement() {
    this.wristRotationPidConstants.retrieveDashboard(this.wristRotationPID);
    if (this.wristSetPoint > Constants.Wrist.maxAngle) {
      this.wristSetPoint = Constants.Wrist.maxAngle;
      SmartDashboard.putString("wrist limit", "MAX EXCEEDED");
    } else if (wristSetPoint < Constants.Wrist.minAngle) {
      this.wristSetPoint = Constants.Wrist.minAngle;
      SmartDashboard.putString("wrist limit", "MIN EXCEEDED");
    } else {
      SmartDashboard.putString("wrist limit", "none");
    }
    double power = 0;
    power = this.wristRotationPID.calculate(this.getEncoderPosition(),
        this.wristSetPoint);

    return power;
  }

  @Override
  public void periodic() {
    double power = this.handleMovement();

    this.wristMotor.set(power);
    SmartDashboard.putNumber("Wrist relative encoder", this.getEncoderPosition());
    SmartDashboard.putNumber("Wrist pid output", power);
    SmartDashboard.putNumber("Wrist absolute encoder", this.getAbsoluteEncoder());
    SmartDashboard.putNumber("Wrist setpoint", this.wristSetPoint);
    SmartDashboard.putBoolean("wrist end", this.atSetpoint());
    SmartDashboard.putBoolean("beambreak", this.breambreak.get());

    // Color detectedColor = colorSensor.getColor();

    // int proximity = colorSensor.getProximity();
    // SmartDashboard.putNumber("Red", detectedColor.red);
    // SmartDashboard.putNumber("Green", detectedColor.green);
    // SmartDashboard.putNumber("Blue", detectedColor.blue);

    // SmartDashboard.putNumber("Proximity", proximity);
    // PieceType gamePieceType = this.getGamePieceType();

    // switch (gamePieceType) {
    // case AIR:
    // SmartDashboard.putString("color sensor", "Nothing - AIR");
    // break;
    // case CONE:
    // SmartDashboard.putString("color sensor", "CONE");
    // break;
    // case CUBE:
    // SmartDashboard.putString("color sensor", "CUBE");
    // break;
    // default:
    // break;
    // }
  }
}
