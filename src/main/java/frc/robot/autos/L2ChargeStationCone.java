package frc.robot.autos;

import java.util.HashMap;
import java.util.List;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Balance;
import frc.robot.commands.presets.ConeL2;
import frc.robot.commands.presets.ConeL2Score;
import frc.robot.commands.presets.ConeTipped;
import frc.robot.commands.presets.Rest;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

public class L2ChargeStationCone implements AutoImpl {
        private final PoseEstimator poseEstimator;
        private final Swerve swerve;
        private final Arm arm;
        private final Wrist wrist;
        private final LEDs leds;
        private final SwerveAutoBuilder autoBuilder;
        private final List<PathPlannerTrajectory> pathGroup;

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
                                new PathConstraints(5, 5));

                HashMap<String, Command> eventMap = new HashMap<>();
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

        public Pose2d getInitialHolonomicPose() {
                return pathGroup.get(0).getInitialHolonomicPose();
        }

        public Command getCommand() {
                return new SequentialCommandGroup(
                                new Rest(arm, wrist, leds),
                                new ConeL2(arm, wrist, leds),
                                autoBuilder.fullAuto(pathGroup), new Balance(swerve, leds));
        }
}
