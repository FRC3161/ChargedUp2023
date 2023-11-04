package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LEDs;

public class MoveArm extends CommandBase {
    private Arm arm;
    private double angle;
    private LEDs leds;

    public MoveArm(Arm arm, double angle, LEDs leds) {
        this.arm = arm;
        this.angle = angle;
        this.leds = leds;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        this.leds.set(Constants.LEDConstants.solidOrange);
        this.arm.setArmSetpoint(this.angle);
    }

    @Override
    public void end(boolean interrupted) {
        this.leds.set(Constants.LEDConstants.off);
    }

    @Override
    public boolean isFinished() {
        return this.arm.atSetpoint();
    }
}
