package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.GamePieceLevel;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Wrist;

public class ConeL1 extends ParallelCommandGroup {
    public ConeL1(Arm arm, Wrist wrist, LEDs leds) {
        addCommands(new InstantCommand(() -> {
            wrist.gamePieceLevel = GamePieceLevel.L1;
        }), new MoveArm(arm, -0.728981, leds).withTimeout(Constants.Arm.commandTimeout),
                new MoveWrist(wrist, -0.810281, leds).withTimeout(Constants.Wrist.commandTimeout));
    }
}
