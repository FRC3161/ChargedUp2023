package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Wrist;

public class ConeTipped extends ParallelCommandGroup {
    public ConeTipped(Arm arm, Wrist wrist, LEDs leds) {
        addCommands(new MoveArm(arm, -49.393056, leds).withTimeout(Constants.Arm.commandTimeout),
                new MoveWrist(wrist, 1.021737, leds).withTimeout(Constants.Wrist.commandTimeout));
    }
}
