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

public class CubeL3 extends ParallelCommandGroup {
    public CubeL3(Arm arm, Wrist wrist, LEDs leds) {
        addCommands(
                new InstantCommand(() -> {
                    wrist.gamePieceLevel = GamePieceLevel.L3;
                }),
                new MoveArm(arm, 17.462, leds).withTimeout(Constants.Arm.commandTimeout),
                new MoveWrist(wrist, 1.584508, leds).withTimeout(Constants.Wrist.commandTimeout));
    }
}
