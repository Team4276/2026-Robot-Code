package frc.team4276.frc2026.field;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
  public static final AprilTagFieldLayout apriltagLayout = AprilTagFieldLayout
      .loadField(AprilTagFields.k2025ReefscapeWelded);

  public static final double fieldLength = Units.inchesToMeters(690.875958);
  public static final double fieldWidth = Units.inchesToMeters(317);
  public static final Translation2d fieldCenter = new Translation2d(Units.inchesToMeters(345.437979),
      Units.inchesToMeters(158.5));

}
