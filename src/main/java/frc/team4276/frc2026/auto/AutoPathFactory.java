package frc.team4276.frc2026.auto;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.team4276.lib.path.ChoreoUtil;

public class AutoPathFactory {
    public static Trajectory<SwerveSample> getMizu(boolean mirrorLengthWise) {
        return ChoreoUtil.getChoreoTrajectory("e_Mizu", mirrorLengthWise);
    }

    public static Trajectory<SwerveSample> getChizuru(boolean mirrorLengthWise) {
        return ChoreoUtil.getChoreoTrajectory("e_Chizuru", mirrorLengthWise);
    }

    public static Trajectory<SwerveSample> getChizu(boolean isDream, boolean mirrorLengthWise) {
        if (isDream) {
            return ChoreoUtil.getChoreoTrajectory("a_YumeChizu", mirrorLengthWise);

        }

        return ChoreoUtil.getChoreoTrajectory("a_Chizu", mirrorLengthWise);
    }

    public static Trajectory<SwerveSample> getCheesu(boolean isDream, boolean mirrorLengthWise) {
        if (isDream) {
            return ChoreoUtil.getChoreoTrajectory("b_YumeCheesu", mirrorLengthWise);

        }

        return ChoreoUtil.getChoreoTrajectory("b_Cheesu", mirrorLengthWise);
    }

    public static Trajectory<SwerveSample> getYOUzu(boolean isDream, boolean mirrorLengthWise) {
        if (isDream) {
            return ChoreoUtil.getChoreoTrajectory("c_YumeYOUzu", mirrorLengthWise);

        }

        return ChoreoUtil.getChoreoTrajectory("c_YOUzu", mirrorLengthWise);
    }

    public static Trajectory<SwerveSample> getVanilla() {
        return ChoreoUtil.getChoreoTrajectory("d_Vanilla");
    }

    public static Trajectory<SwerveSample> getVanilleft() {
        return ChoreoUtil.getChoreoTrajectory("d_Vanilleft");
    }

    public static Trajectory<SwerveSample> getVaniright() {
        return ChoreoUtil.getChoreoTrajectory("d_Vaniright");
    }

    public static Trajectory<SwerveSample> getMint() {
        return ChoreoUtil.getChoreoTrajectory("d_Mint");
    }
}
