package frc.robot.autos;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.PieceType;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.presets.ConeL2;
import frc.robot.commands.presets.ConeStanding;
import frc.robot.commands.presets.CubeIntake;
import frc.robot.commands.presets.CubeL2;
import frc.robot.commands.presets.CubeL2Score;
import frc.robot.commands.presets.Rest;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

public class CableSideL2 extends AutoBase {
  public CableSideL2(Swerve swerve, PoseEstimator poseEstimator, Arm arm, Wrist wrist,
      LEDs leds) {
    this.poseEstimator = poseEstimator;
    this.swerve = swerve;
    this.arm = arm;
    this.wrist = wrist;
    this.leds = leds;

    pathGroup = PathPlanner.loadPathGroup("cableside",
        new PathConstraints(1.5,
            1.5));
    pathGroup_red = PathPlanner.loadPathGroup("cableside_red",
        new PathConstraints(1.5,
            1.5));

    eventMap = new HashMap<>();
    eventMap.put("outake", new SequentialCommandGroup(new IntakeOut(arm, wrist, leds), new Rest(arm, wrist, leds)));
    eventMap.put("rest", new Rest(arm, wrist, leds));
    eventMap.put("intakecube", new IntakeIn(arm, wrist, PieceType.CUBE, leds));
    eventMap.put("cubeintakeposition", new CubeIntake(arm, wrist, leds));
    eventMap.put("cubel2", new CubeL2(arm, wrist, leds));
    eventMap.put("standingcone", new ConeStanding(arm, wrist, leds));
    eventMap.put("intakecone", new IntakeIn(arm, wrist, PieceType.CONE, leds));
    eventMap.put("conel2", new ConeL2(arm, wrist, leds));

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
        new Rest(arm, wrist, leds),
        new ConeL2(arm, wrist, leds),
        autoBuilder.fullAuto(getPathGroup()),
        new IntakeOut(arm, wrist, leds));
  }
}
