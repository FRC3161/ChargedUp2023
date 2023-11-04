/*
 * Copyright (c) 2023 Tigerbotics and it's members. All rights reserved.
 * This work is licensed under the terms of the GNU GPLv3 license
 * found in the root directory of this project.
 */
package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;

/**
 * Utility functions for flipping from the blue to red alliance. By default, all
 * translations and
 * poses in {@link FieldConstants} are stored with the origin at the rightmost
 * point on the blue
 * alliance wall.
 *
 * @author Jonah | Mechanical Advantage 6328
 * @author Jeffrey Morris | Tigerbotics 7125
 */
public class AllianceFlip {

    /** @return Whether or not the pose should be flipped, as blue is default. */
    private static boolean shouldFlip() {
        return DriverStation.getAlliance() == Alliance.Red;
    }

    /**
     * Flips a translation to the correct side of the field based on the current
     * alliance color.
     */
    public static Translation2d apply(Translation2d translation) {
        if (shouldFlip())
            return new Translation2d(
                    FieldConstants.fieldLength - translation.getX(), FieldConstants.fieldWidth - translation.getY());
        return translation;
    }

    /** Flips a rotation based on the current alliance color. */
    public static Rotation2d apply(Rotation2d rotation) {
        return rotation;

        // if (shouldFlip())
        // return new Rotation2d(-rotation.getCos(), rotation.getSin());
        // return rotation;
    }

    /**
     * Flips a pose to the correct side of the field based on the current alliance
     * color.
     */
    public static Pose2d apply(Pose2d pose) {
        if (shouldFlip())
            return new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()));
        return pose;
    }
}