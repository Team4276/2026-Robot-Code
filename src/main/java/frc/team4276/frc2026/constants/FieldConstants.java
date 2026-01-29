package frc.team4276.frc2026.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
    public static final AprilTagFieldLayout apriltagLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2026RebuiltWelded);

    public static final double fieldLength = apriltagLayout.getFieldLength();
    public static final double fieldWidth = apriltagLayout.getFieldWidth();

    public static final Translation2d tempHubCenter = new Translation2d(apriltagLayout.getTagPose(26).get().getX() + Units.inchesToMeters(47.0) / 2.0, fieldWidth / 2);
    public static final Translation2d tempFerryTarget = Translation2d.kZero;
}
