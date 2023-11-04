package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotModes;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Swerve;

public class GoToPosition extends CommandBase {
  private final Transform3d position;
  private final Swerve swerve;
  private final PoseEstimator poseEstimator;
  private final ProfiledPIDController xController = Constants.Vision.translationController;
  private final ProfiledPIDController yController = Constants.Vision.translationController;
  private final ProfiledPIDController rotationController = Constants.Vision.rotationController;

  public GoToPosition(
      Swerve swerve,
      PoseEstimator poseEstimator,
      Transform3d position) {
    this.swerve = swerve;
    this.poseEstimator = poseEstimator;
    this.position = position;

    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    rotationController.setTolerance(Units.degreesToRadians(3));
    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    var robotPose = this.poseEstimator.currentPose();
    rotationController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());

    rotationController.setGoal(this.position.getRotation().getY());
    xController.setGoal(this.position.getX());
    yController.setGoal(this.position.getY());
  }

  @Override
  public void execute() {
    var robotPose2d = this.poseEstimator.currentPose();

    var xSpeed = xController.calculate(robotPose2d.getX());
    if (xController.atGoal()) {
      xSpeed = 0;
    }

    var ySpeed = yController.calculate(robotPose2d.getY());
    if (yController.atGoal()) {
      ySpeed = 0;
    }

    var omegaSpeed = rotationController.calculate(robotPose2d.getRotation().getRadians());
    if (rotationController.atGoal()) {
      omegaSpeed = 0;
    }

    if (Constants.robotMode == RobotModes.Debug) {
      SmartDashboard.putNumber("X target", xSpeed);
      SmartDashboard.putNumber("Y target", ySpeed);
      SmartDashboard.putNumber("Omega target", omegaSpeed);
    }

    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed,
            omegaSpeed,
            robotPose2d.getRotation()));

    // this.swerve.setModuleStates(swerveModuleStates);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.brake();
  }

  @Override
  public boolean isFinished() {
    return this.xController.atGoal() && this.yController.atGoal() && this.rotationController.atGoal();
  }

}