package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Wrist;

public class ConeL2Sequential extends SequentialCommandGroup {
    public ConeL2Sequential(Arm arm, Wrist wrist, LEDs leds) {
        addCommands(
                new MoveArm(arm, 31, leds).withTimeout(Constants.Arm.commandTimeout),
                new MoveWrist(wrist, -0.649656, leds).withTimeout(Constants.Wrist.commandTimeout));
    }
}
