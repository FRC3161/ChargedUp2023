package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeOut;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Wrist;

public class CubeL3Score extends SequentialCommandGroup {
    public CubeL3Score(Arm arm, Wrist wrist, LEDs leds) {
        addCommands(
                new CubeL3(arm, wrist, leds),
                Commands.waitSeconds(1),
                new IntakeOut(arm, wrist, leds), new Rest(arm, wrist, leds));
    }
}
