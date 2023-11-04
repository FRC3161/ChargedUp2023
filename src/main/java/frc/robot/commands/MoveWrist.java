package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Wrist;

public class MoveWrist extends CommandBase {
    private Wrist wrist;
    private double angle;
    private LEDs leds;
    public int done = 0;

    public MoveWrist(Wrist wrist, double angle, LEDs leds) {
        this.wrist = wrist;
        this.angle = angle;
        this.leds = leds;
        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        this.leds.set(Constants.LEDConstants.solidOrange);
        this.wrist.setWristSetpoint(this.angle);
    }

    @Override
    public void end(boolean interrupted) {
        this.leds.set(Constants.LEDConstants.off);
    }

    @Override
    public boolean isFinished() {
        return this.wrist.atSetpoint();
    }
}
