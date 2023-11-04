package frc.lib.logging;

import java.util.List;

import com.revrobotics.CANSparkMax;


public interface SysIdDrivetrain {
    public List<CANSparkMax> getLeftMotors();

    public List<CANSparkMax> getRightMotors();

    public double getLeftPosition();

    public double getRightPosition();

    public double getLeftVelocity();

    public double getRightVelocity();

    public double getGyroAngle();

    public double getGyroRate();
}