package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LEDs;

public class SetColor extends CommandBase {
    private LEDs leds;
    private double color;

    public SetColor(LEDs leds, double color) {
        this.leds = leds;
        this.color = color;

        addRequirements(leds);
    }

    @Override
    public void initialize() {
        this.leds.set(color);
    }

    @Override
    public void end(boolean interrupted) {
        this.leds.set(Constants.LEDConstants.off);
    }
}
