package frc.robot.autos;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.PieceType;
import frc.robot.commands.IntakeIn;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.presets.ConeStanding;
import frc.robot.commands.presets.CubeL3Score;
import frc.robot.commands.presets.Rest;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

public class CableSideL3 extends AutoBase {
  public CableSideL3(Swerve swerve, PoseEstimator poseEstimator, Arm arm, Wrist wrist,
      LEDs leds) {
    this.poseEstimator = poseEstimator;
    this.swerve = swerve;
    this.arm = arm;
    this.wrist = wrist;
    this.leds = leds;

    pathGroup = PathPlanner.loadPathGroup("cableside",
        new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    pathGroup_red = PathPlanner.loadPathGroup("cableside_red",
        new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));

    eventMap = new HashMap<>();
    eventMap.put("standingcone", new ConeStanding(arm, wrist, leds));
    eventMap.put("intakecone", new IntakeIn(arm, wrist, PieceType.CONE, leds));

    autoBuilder = new SwerveAutoBuilder(poseEstimator::currentPose,
        poseEstimator::setCurrentPose,
        Constants.Swerve.swerveKinematics,
        new PIDConstants(Constants.AutoConstants.translationPID.p,
            Constants.AutoConstants.translationPID.i,
            Constants.AutoConstants.translationPID.d),
        new PIDConstants(Constants.AutoConstants.rotationPID.p,
            Constants.AutoConstants.rotationPID.i,
            Constants.AutoConstants.rotationPID.d),
        swerve::setModuleStates,
        eventMap,
        false,
        swerve);
  }

  @Override
  public Command getCommand() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          this.wrist.currentPiece = PieceType.CUBE;
        }),
        new Rest(arm, wrist, leds),
        new CubeL3Score(arm, wrist, leds),
        autoBuilder.fullAuto(getPathGroup()));
  }
}
