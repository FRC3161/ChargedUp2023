package frc.lib.util;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;

public class Flipper {
  public static boolean shouldFlip() {
    return DriverStation.getAlliance() == Alliance.Red;
  }

  public static Pose2d flipIfShould(Pose2d pose) {
    if (shouldFlip()) {
      return allianceFlip(pose);
    }
    return pose;
  }

  public static Rotation2d flipIfShould(Rotation2d rotation2d) {
    if (shouldFlip()) {
      return allianceFlip(rotation2d);
    }
    return rotation2d;
  }

  public static Rotation2d allianceFlip(Rotation2d rotation2d) {
    return new Rotation2d(-rotation2d.getCos(), rotation2d.getSin());
  }

  public static Pose2d allianceFlip(Pose2d pose) {
    return new Pose2d(
        new Translation2d(
            FieldConstants.fieldLength - pose.getTranslation().getX(),
            FieldConstants.fieldWidth - pose.getTranslation().getY()),
        allianceFlip(pose.getRotation()));
  }

  public static PathPlannerTrajectory allianceFlip(PathPlannerTrajectory trajectory) {
    List<State> newStates = new ArrayList<>();

    for (State s : trajectory.getStates()) {
      PathPlannerState state = (PathPlannerState) s;

      newStates.add(allianceFlip(state));
    }
    return new PathPlannerTrajectory(
        newStates,
        trajectory.getMarkers(),
        trajectory.getStartStopEvent(),
        trajectory.getEndStopEvent(),
        trajectory.fromGUI);
  }

  public static PathPlannerState allianceFlip(PathPlannerState state) {
    Pose2d transformedPose = allianceFlip(state.poseMeters);
    Rotation2d transformedHolonomicRotation = allianceFlip(state.holonomicRotation);

    // Some things are private so just have this do those for us, then we will
    // overwrite what needs changing
    PathPlannerState transformedState = PathPlannerTrajectory.transformStateForAlliance(state, Alliance.Red);
    transformedState.poseMeters = transformedPose;
    transformedState.holonomicRotation = transformedHolonomicRotation;

    return transformedState;
  }
}
