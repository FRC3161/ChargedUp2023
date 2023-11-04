package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class FancyRotation extends CommandBase {
    private final Swerve swerve;

    public FancyRotation(Swerve swerve) {
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
        this.swerve.setCoR(new Translation2d(1, 0));
    }

    @Override
    public void end(boolean interrupted) {
        this.swerve.resetCoR();
    }
}
