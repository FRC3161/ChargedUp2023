package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.PieceType;
import frc.robot.commands.presets.ConeL1;
import frc.robot.commands.presets.ConeL2;
import frc.robot.commands.presets.ConeL3;
import frc.robot.commands.presets.CubeL1;
import frc.robot.commands.presets.CubeL2;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class Score {
    private Arm arm;
    private Wrist wrist;

    public Score(Arm arm, Wrist wrist) {
        this.arm = arm;
        this.wrist = wrist;
    }

    public ParallelCommandGroup L1() {
        if (wrist.currentPiece == PieceType.CONE) {
            return new ConeL1(arm, wrist);
        } else if (wrist.currentPiece == PieceType.CUBE) {
            return new CubeL1(arm, wrist);
        }

        return new ParallelCommandGroup();
    }

    public ParallelCommandGroup L2() {
        if (wrist.currentPiece == PieceType.CONE) {
            return new ConeL2(arm, wrist);
        } else if (wrist.currentPiece == PieceType.CUBE) {
            return new CubeL2(arm, wrist);
        }

        return new ParallelCommandGroup();
    }

    public ParallelCommandGroup L3() {
        if (wrist.currentPiece == PieceType.CONE) {
            return new ConeL3(arm, wrist);
        } else if (wrist.currentPiece == PieceType.CUBE) {
            // TBD
            return new ParallelCommandGroup();
        }

        return new ParallelCommandGroup();
    }

}
