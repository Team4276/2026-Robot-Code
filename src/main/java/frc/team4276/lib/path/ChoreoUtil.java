package frc.team4276.lib.path;

import java.util.ArrayList;
import java.util.List;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.trajectory.TrajectorySample;

public class ChoreoUtil {
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
