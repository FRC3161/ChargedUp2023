package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LEDs;

public class Blink extends CommandBase {
    private final LEDs leds;
    private final double color;
    private int counter = 0;
    private boolean on;

    public Blink(LEDs leds, double color) {
        this.color = color;
        this.leds = leds;

        addRequirements(leds);
    }

    @Override
    public void initialize() {
        this.leds.set(color);
        this.counter = 0;
        this.on = true;
    }

    @Override
    public void end(boolean interrupted) {
        this.leds.set(Constants.LEDConstants.off);
    }

    @Override
    public void execute() {
        if (counter >= Constants.LEDConstants.blinkPerSecond) {
            this.counter = 0;
            this.leds.set(this.on ? Constants.LEDConstants.off : color);
            this.on = !this.on;
        }
        counter++;
    }
}
