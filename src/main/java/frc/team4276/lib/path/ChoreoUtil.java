package frc.team4276.lib.path;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.trajectory.TrajectorySample;
import frc.team4276.lib.geometry.AllianceFlipUtil;

public class ChoreoUtil {
    private ChoreoUtil() {
    }

    // Assume swerve sample and if not then we screwed ig
    /** Loads and flips trajectory accordingly */
    public static Trajectory<SwerveSample> getChoreoTrajectory(String name) {
        return getChoreoTrajectory(name, false);
    }

    public static Trajectory<SwerveSample> getChoreoTrajectory(
            String name, boolean mirrorLengthwise) {
        Optional<Trajectory<SwerveSample>> uncheckedTraj = Choreo.loadTrajectory(name);
        try {
            var traj = uncheckedTraj.orElseThrow();

            if (mirrorLengthwise) {
                traj = mirrorLengthwise(traj);
            }

            return AllianceFlipUtil.shouldFlip() ? traj.flipped() : traj;
        } catch (Exception e) {
            System.out.println("Failed to load trajectory " + name);
            return new Trajectory<SwerveSample>(name, List.of(), List.of(), List.of());
        }
    }

    // Assume swerve sample and if not then we screwed ig
    /** Loads and flips trajectory accordingly */
    public static Trajectory<SwerveSample> getChoreoTrajectory(String name, int split) {
        return getChoreoTrajectory(name, false, split);
    }

    public static Trajectory<SwerveSample> getChoreoTrajectory(
            String name, boolean mirrorLengthwise, int split) {
        Optional<Trajectory<SwerveSample>> uncheckedTraj = Choreo.loadTrajectory(name);
        try {
            var traj = uncheckedTraj.orElseThrow().getSplit(split).orElseThrow();

            if (mirrorLengthwise) {
                traj = mirrorLengthwise(traj);
            }

            return AllianceFlipUtil.shouldFlip() ? traj.flipped() : traj;
        } catch (Exception e) {
            System.out.println("Failed to load split " + split + " of trajectory " + name);
            return new Trajectory<SwerveSample>(name, List.of(), List.of(), List.of());
        }
    }

    public static Trajectory<SwerveSample> mirrorLengthwise(Trajectory<?> trajectory) {
        List<SwerveSample> mirroredStates = new ArrayList<>();

        for (var state : trajectory.samples()) {
            mirroredStates.add(mirrorLengthwise(state));
        }
        return new Trajectory<SwerveSample>("", mirroredStates, List.of(), List.of());
    }

    private static final double[] dummyList = { 0.0, 0.0, 0.0, 0.0 };

    public static SwerveSample mirrorLengthwise(TrajectorySample<?> state) {
        var flipped = new SwerveSample(
                state.getTimestamp(),
                state.getPose().getX(),
                state.getPose().getY(),
                state.getPose().getRotation().getRadians(),
                state.getChassisSpeeds().vxMetersPerSecond,
                state.getChassisSpeeds().vyMetersPerSecond,
                state.getChassisSpeeds().omegaRadiansPerSecond, 0, 0, 0, dummyList, dummyList);

        return flipped;
    }
}
