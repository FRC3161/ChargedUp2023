package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Wrist;

public class ConeStanding extends ParallelCommandGroup {
    public ConeStanding(Arm arm, Wrist wrist, LEDs leds) {
        addCommands(
                new MoveArm(arm, -8.2, leds).withTimeout(Constants.Arm.commandTimeout),
                new MoveWrist(wrist, -0.95, leds).withTimeout(Constants.Wrist.commandTimeout));
    }
}
