// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.logging;

import com.revrobotics.CANSparkMax;

public interface SysIdMechanism {
    public CANSparkMax getMotor();

    public double getPosition();

    public double getVelocity();
}