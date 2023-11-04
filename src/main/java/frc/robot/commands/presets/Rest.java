package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Wrist;

public class Rest extends ParallelCommandGroup {
    private static double armSetpoint = -58.3;
    private static double wristSetpoint = 3.0;

    public Rest(Arm arm, Wrist wrist, LEDs leds) {
        addCommands(
                new MoveArm(arm, armSetpoint, leds).withTimeout(Constants.Arm.commandTimeout),
                new MoveWrist(wrist, wristSetpoint, leds).withTimeout(Constants.Wrist.commandTimeout));
    }

    public static void forceSet(Arm arm, Wrist wrist) {
        arm.setArmSetpoint(armSetpoint);
        wrist.setWristSetpoint(wristSetpoint);
    }
}
