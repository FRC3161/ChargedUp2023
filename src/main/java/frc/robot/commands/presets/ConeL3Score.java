package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PieceType;
import frc.robot.commands.IntakeOut;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Wrist;

public class ConeL3Score extends SequentialCommandGroup {
    public ConeL3Score(Arm arm, Wrist wrist, LEDs leds) {
        wrist.currentPiece = PieceType.CONE;
        addCommands(
                new Rest(arm, wrist, leds),
                new ConeL2(arm, wrist, leds),
                new ConeL3(arm, wrist, leds),
                Commands.waitSeconds(0.8),
                new IntakeOut(arm, wrist, leds), new Rest(arm, wrist, leds));
    }
}
