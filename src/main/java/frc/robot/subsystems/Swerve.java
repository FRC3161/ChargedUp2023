package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.PIDConstants;
import frc.lib.util.SVAConstants;
import frc.lib.util.SwerveModule;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
  public SwerveModule[] mSwerveMods;
  public Pigeon2 gyro;
  public Rotation2d orientationWhenReleased;
  private double rotationControllerSpeed = 0.0;
  public final PIDController robotRotationPID;
  private boolean wasRotationZero = true;
  private boolean wasTranslationZero = true;
  private long defenseDelayStart;
  private Translation2d centerOfRotation = new Translation2d();

  private double pitchOffset = 0;
  private double rollOffset = 0;

  /* Auto variables */
  private PIDConstants drivePIDConstants = Constants.Swerve.drivePID;
  private SVAConstants driveSVAConstants = Constants.Swerve.driveSVA;
  public PIDConstants autoTranslationConstants = Constants.AutoConstants.translationPID;
  public PIDConstants autoRotationConstants = Constants.AutoConstants.rotationPID;

  public Swerve() {
    /* Gyro setup */
    gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.Swerve.pigeonCanBUS);
    gyro.configFactoryDefault();
    this.pitchOffset = gyro.getPitch();
    this.rollOffset = gyro.getRoll();
    zeroGyro();

    /* Custom PID controllers setup */
    this.robotRotationPID = Constants.Swerve.robotRotationPID.getController();
    this.robotRotationPID.enableContinuousInput(-180, 180);
    this.robotRotationPID.setTolerance(2);

    driveSVAConstants.sendDashboard("Module");
    drivePIDConstants.sendDashboard("Module Velocity");
    autoTranslationConstants.sendDashboard("Auto Translation");
    autoRotationConstants.sendDashboard("Auto Rotation");

    /* Swerve modules setup */
    mSwerveMods = new SwerveModule[] {
        new SwerveModule(0, Constants.Swerve.Mod0.constants),
        new SwerveModule(1, Constants.Swerve.Mod1.constants),
        new SwerveModule(2, Constants.Swerve.Mod2.constants),
        new SwerveModule(3, Constants.Swerve.Mod3.constants)
    };

  }

  /**
   * Updates the robots position hold target when the rotation axis is not being
   * used
   * 
   * @param rotation the value of the rotation axis
   */
  public void rotationUpdate(double rotation) {
    if (this.rotationControllerSpeed != 0.0) {
      this.resetHold();
    }

    this.rotationControllerSpeed = rotation;
  }

  /**
   * The main function used for driving the robot
   * 
   * @param translation
   * @param rotation
   * @param fieldRelative
   * @param isOpenLoop    True -> no PID, False -> PID
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop,
      boolean isDefense, boolean defenseProtectionOverride) {
    double missalignment = 0;
    this.rotationUpdate(rotation);

    // Clamp the output to the maxSpeed so that the robot doesn't make hole in the
    // wall :D
    if (defenseProtectionOverride || (isDefense && this.shouldDefense(rotation, translation))) {
      missalignment = MathUtil.clamp(
          this.robotRotationPID.calculate(getYaw().getDegrees() % 360,
              this.orientationWhenReleased.getDegrees() % 360),
          -Constants.Swerve.maxSpeed, Constants.Swerve.maxSpeed);
      missalignment = 0;
    }

    SmartDashboard.putNumber("Missalignment speed PID", missalignment);

    // get the states for each module
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(),
                rotation + (Constants.Swerve.driveInvert ? missalignment : -missalignment),
                getYaw())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation),
        this.centerOfRotation);

    // Get rid of tiny tiny movements in the wheels to have more consistent driving
    // experience
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    // set the states for each module
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  // public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean
  // isFirstPath) {
  // var commandGroup = new SequentialCommandGroup();

  // commandGroup.addCommands(new InstantCommand(() -> {
  // // Reset odometry for the first path you run during auto
  // if (isFirstPath) {
  // this.resetOdometry(traj.getInitialHolonomicPose());
  // }
  // }),
  // new PPSwerveControllerCommand(
  // traj,
  // this::getPose, // Pose supplier
  // Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
  // Constants.AutoConstants.translationPID.getController(),
  // Constants.AutoConstants.translationPID.getController(),
  // Constants.AutoConstants.rotationPID.getController(),
  // this::setModuleStates, // Module states consumer
  // this // Requires this drive subsystem
  // ));
  // return commandGroup;
  // }

  /**
   * 1. if rotation is 0 while it wasn't before this iteration, then start a
   * counter that is set for 1 second and after defense should be true based on
   * other rules
   * 2. if translation norm is 0 while it wasn't before this iteration, then start
   * a counter that is set for 1 second and after defense should be true based on
   * other rules
   * 3. if both rotation and translation are more than 0, then defense should be
   * false
   * 4. if only translation norm is more than 0, then defense should be true
   * 5. if only rotation is more than 0, then defense should be false
   * 
   * @param rotation    joystick rotation
   * @param translation joystick translation value
   * @return should defense or not
   */
  public boolean shouldDefense(double rotation, Translation2d translation) {
    boolean defense = false;
    double norm = translation.getNorm();

    if (rotation == 0 && !this.wasRotationZero) {
      this.defenseDelayStart = System.currentTimeMillis();
      defense = true;
    }
    if (norm == 0 && !this.wasTranslationZero) {
      this.defenseDelayStart = System.currentTimeMillis();
      defense = true;
    }
    if (rotation == 0 && norm == 0) {
      defense = true;
    }

    if (rotation != 0 && norm != 0) {
      defense = false;
    }
    if (rotation != 0 && norm == 0) {
      defense = false;
    }
    if (rotation == 0 && norm != 0) {
      defense = true;
    }

    if (defense && System.currentTimeMillis() - this.defenseDelayStart < Constants.Swerve.defenseDelay) {
      defense = false;
    }

    this.wasRotationZero = (rotation == 0);
    this.wasTranslationZero = (norm == 0);

    return defense;
  }

  /**
   * The function used in auto to set the states for each module, does not have
   * vision tracking and position hold.
   * 
   * @param desiredStates the desired state foe each module
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  /** Reset the module encoder values */
  public void resetModuleZeros() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  /**
   * Get the current state of the modules
   * 
   * @return state of the modules
   */
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }

    return positions;
  }

  public SwerveModulePosition[] getRedPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getRedPosition();
    }

    return positions;
  }

  /** Reset the gyro and position hold */
  public void zeroGyro() {
    gyro.setYaw(0);
    this.orientationWhenReleased = Rotation2d.fromDegrees(0);

    // CustomThreads.setTimeout(() -> {
    // this.orientationWhenReleased = Rotation2d.fromDegrees(0);
    // }, 20);

  }

  /** reset position hold value to whatever the gyro is */
  public void resetHold() {
    this.orientationWhenReleased = getYaw();
  }

  /**
   * Set the positionHold value to whatever angle is given
   * 
   * @param angle the angle to point the robot towards and hold
   */
  public void setHold(double angle) {
    this.orientationWhenReleased = Rotation2d.fromDegrees(360 - angle);
  }

  public void brake() {
    SwerveModule[] modules = this.mSwerveMods;
    modules[0].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
    modules[1].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(315)), true);
    modules[2].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(315)), true);
    modules[3].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
  }

  /**
   * get the orientation of the robot
   * 
   * @return the orientation of the robot
   */
  public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : Rotation2d.fromDegrees(gyro.getYaw());
  }

  /**
   * Get the pitch from gyro
   * 
   * @return the pitch of the robot
   */
  public Rotation2d getPitch() {
    return Rotation2d.fromDegrees(gyro.getPitch() - this.pitchOffset);
  }

  public Rotation2d getRoll() {
    return Rotation2d.fromDegrees(gyro.getRoll() - this.rollOffset);
  }

  public void resetCoR() {
    this.centerOfRotation = new Translation2d();
  }

  public void setCoR(Translation2d translation) {
    this.centerOfRotation = translation;
  }

  public void retrieveAutoConstants() {
    this.autoTranslationConstants.retrieveDashboard();
    this.autoRotationConstants.retrieveDashboard();
    this.drivePIDConstants.retrieveDashboard();
    SimpleMotorFeedforward newSVA = this.driveSVAConstants.retrieveDashboard();

    for (SwerveModule mod : mSwerveMods) {
      this.drivePIDConstants.applyPID(mod.driveController);
      mod.feedforward = newSVA;
    }
  }

  @Override
  public void periodic() {
    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(Constants.Swerve.moduleNames[mod.moduleNumber] +
          "speed",
          mod.getSpeed());
    }
    SmartDashboard.putNumber("roll", getRoll().getDegrees());
    SmartDashboard.putNumber("pitch", getPitch().getDegrees());
    SmartDashboard.putNumber("yaw", getYaw().getDegrees());
    // SmartDashboard.putNumber("orientationHold",
    // this.orientationWhenReleased.getDegrees());
  }
}
