package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Wrist;

public class RestAuto extends ParallelCommandGroup {
    public RestAuto(Arm arm, Wrist wrist, LEDs leds) {
        addCommands(
                new MoveArm(arm, -54, leds).withTimeout(Constants.Arm.commandTimeout),
                new MoveWrist(wrist, 2.91, leds).withTimeout(Constants.Wrist.commandTimeout));
    }
}
