package frc.robot.autos;

import java.util.HashMap;
import java.util.List;

import javax.print.attribute.standard.PrinterIsAcceptingJobs;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.PieceType;
import frc.robot.commands.Balance;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.presets.ConeL2;
import frc.robot.commands.presets.ConeL2Score;
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

public class Farside25Balance extends AutoBase {
  public Farside25Balance(Swerve swerve, PoseEstimator poseEstimator, Wrist wrist, Arm arm, LEDs leds) {
    this.swerve = swerve;
    this.poseEstimator = poseEstimator;
    this.wrist = wrist;
    this.arm = arm;
    this.leds = leds;

    pathGroup = PathPlanner.loadPathGroup("Farside2.5+Balance",
        new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    pathGroup_red = PathPlanner.loadPathGroup("Farside2.5+Balance_red",
        new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));

    HashMap<String, Command> eventMap = new HashMap<>();
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

  public Command getCommand() {
    return new SequentialCommandGroup(
        new Rest(arm, wrist, leds),
        new ConeL2(arm, wrist, leds),
        autoBuilder.fullAuto(getPathGroup()),
        new Balance(swerve, leds));
  }
}
