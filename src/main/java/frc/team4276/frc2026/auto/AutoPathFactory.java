package frc.team4276.frc2026.auto;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.team4276.lib.path.ChoreoUtil;

public class AutoPathFactory {
    // Trajectory caching
    // Path Generation

    public static Trajectory<SwerveSample> getVanilla(){
        return ChoreoUtil.getChoreoTrajectory("Vanilla");
    }

    public static Trajectory<SwerveSample> getVanilleft(){
        return ChoreoUtil.getChoreoTrajectory("Vanilleft");
    }

    public static Trajectory<SwerveSample> getVaniright(){
        return ChoreoUtil.getChoreoTrajectory("Vaniright");
    }
}
