package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PoseEstimator extends SubsystemBase {
  private final Swerve swerve;
  private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;
  private final Field2d field2d = new Field2d();
  private AprilTagFieldLayout aprilTagFieldLayout;
  private final PhotonCamera rightCamera;
  private final PhotonCamera leftCamera;

  public PoseEstimator(Swerve swerve, PhotonCamera rightCamera, PhotonCamera leftCamera) {
    this.swerve = swerve;
    this.rightCamera = rightCamera;
    this.leftCamera = leftCamera;

    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (Exception e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      aprilTagFieldLayout = null;
    }

    this.swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics,
        this.swerve.getYaw(), this.swerve.getPositions(), new Pose2d(), Constants.Swerve.stateStdDevs,
        Constants.Vision.visionMeasurementStdDevs);

    SmartDashboard.putData("Field", field2d);
  }

  public Optional<Pose3d> getTagFromCamera(PhotonCamera camera) {
    var result = this.rightCamera.getLatestResult();
    if (!result.hasTargets())
      return Optional.empty();

    var id = result.getBestTarget().getFiducialId();
    if ((id == -1 || id == 4 || id == 5) && result.getBestTarget().getPoseAmbiguity() < 0.3)
      return Optional.empty();

    var tag = aprilTagFieldLayout.getTagPose(id);
    if (!tag.isEmpty()) {
      return Optional.of(tag.get()
          .plus(result.getBestTarget().getBestCameraToTarget().inverse()));
    }
    return tag;
  }

  @Override
  public void periodic() {
    this.swerveDrivePoseEstimator.update(this.swerve.getYaw(), this.swerve.getPositions());

    SmartDashboard.putString("Estimated Pose", this.getFormattedPose());
    field2d.setRobotPose(currentPose());

    var rightResult = this.rightCamera.getLatestResult();
    if (rightResult.hasTargets()) {
      var rightId = rightResult.getBestTarget().getFiducialId();
      if (rightId != -1 && rightId != 4 && rightId != 5 && rightResult.getBestTarget().getPoseAmbiguity() < 0.3) {
        var rightTag = aprilTagFieldLayout.getTagPose(rightId);
        if (rightTag.isPresent()) {

          this.swerveDrivePoseEstimator.addVisionMeasurement(
              rightTag.get()
                  .plus(rightResult.getBestTarget().getBestCameraToTarget().inverse())
                  .plus(Constants.Vision.cameraToRobot)
                  .toPose2d(),
              rightResult.getTimestampSeconds());
        }
      }
    }

    var leftResult = this.leftCamera.getLatestResult();
    if (leftResult.hasTargets()) {
      var leftId = leftResult.getBestTarget().getFiducialId();
      if (leftId != -1 && leftId != 4 && leftId != 5 && leftResult.getBestTarget().getPoseAmbiguity() < 0.3) {
        var leftTag = aprilTagFieldLayout.getTagPose(leftId);
        if (leftTag.isPresent()) {

          this.swerveDrivePoseEstimator.addVisionMeasurement(
              leftTag.get()
                  .plus(leftResult.getBestTarget().getBestCameraToTarget().inverse())
                  .plus(Constants.Vision.leftCameraToRobot)
                  .toPose2d(),
              leftResult.getTimestampSeconds());
        }
      }
    }
    // var rightCameraTag = getTagFromCamera(rightCamera);
    // var leftCameraTag = getTagFromCamera(leftCamera);

    // if (!rightCameraTag.isEmpty()) {

    // }

  }

  public Pose2d currentPose() {
    return this.swerveDrivePoseEstimator.getEstimatedPosition();
  }

  public void setCurrentPose(Pose2d newPose) {
    this.swerveDrivePoseEstimator.resetPosition(
        this.swerve.getYaw(),
        this.swerve.getPositions(),
        newPose);
  }

  private String getFormattedPose() {
    var pose = currentPose();
    return String.format("(%.2f, %.2f) %.2f degrees",
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }
}