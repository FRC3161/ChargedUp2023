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

public class ConeL3 extends ParallelCommandGroup {
    public ConeL3(Arm arm, Wrist wrist, LEDs leds) {
        addCommands(
                new InstantCommand(() -> {
                    wrist.gamePieceLevel = GamePieceLevel.L3;
                    wrist.isCorrecting = true;
                }),
                new MoveArm(arm, 52.779, leds).withTimeout(Constants.Arm.commandTimeout),
                new MoveWrist(wrist, -0.673, leds).withTimeout(Constants.Wrist.commandTimeout));
    }
}
