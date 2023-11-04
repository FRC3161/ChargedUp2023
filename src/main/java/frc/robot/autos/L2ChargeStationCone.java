package frc.robot.autos;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Balance;
import frc.robot.commands.presets.ConeL2;
import frc.robot.commands.presets.ConeL2Score;
import frc.robot.commands.presets.Rest;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

public class L2ChargeStationCone extends AutoBase {
    public L2ChargeStationCone(Swerve swerve, PoseEstimator poseEstimator, Arm arm, Wrist wrist,
            LEDs leds) {
        this.poseEstimator = poseEstimator;
        this.swerve = swerve;
        this.arm = arm;
        this.wrist = wrist;
        this.leds = leds;

        pathGroup = PathPlanner.loadPathGroup("l2chargestation",
                new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared),
                new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared),
                new PathConstraints(4, 4));

        pathGroup_red = PathPlanner.loadPathGroup("l2chargestation_red",
                new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared),
                new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared),
                new PathConstraints(4, 4));

        eventMap = new HashMap<>();
        eventMap.put("conel2score", new ConeL2Score(arm, wrist, leds));

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
                autoBuilder.fullAuto(getPathGroup()), new Balance(swerve, leds));
    }
}
