package frc.team4276.frc2026.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team4276.frc2026.FieldConstants;
import frc.team4276.frc2026.RobotContainer;
import frc.team4276.frc2026.RobotState;
import frc.team4276.frc2026.RobotState.VisionState;
import frc.team4276.frc2026.shooter.ShooterConstants.ParamPreset;
import frc.team4276.lib.dashboard.Elastic;
import frc.team4276.lib.dashboard.LoggedTunableNumber;
import frc.team4276.lib.dashboard.Elastic.Notification;
import frc.team4276.lib.dashboard.Elastic.Notification.NotificationLevel;
import frc.team4276.lib.geometry.AllianceFlipUtil;
import frc.team4276.lib.path.ChoreoUtil;

import java.util.List;
import java.util.function.Supplier;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

@SuppressWarnings("unused")
public class AutoFactory {
    private final LoggedTunableNumber preloadShotTime = new LoggedTunableNumber("Auto/PreloadShotTime", 5.0);
    private final LoggedTunableNumber refillShotTime = new LoggedTunableNumber("Auto/RefillShotTime", 5.0);
    private final LoggedTunableNumber fullShotTime = new LoggedTunableNumber("Auto/FullShotTime", 7.5);

    private final LoggedTunableNumber sprinkleWaitTime = new LoggedTunableNumber("Auto/SprinkleWaitTime", 5.0);

    private final LoggedTunableNumber intakeDeployTime = new LoggedTunableNumber("Auto/IntakeDeployTime", 0.5);

    private final LoggedTunableNumber intakeDeployVoltage = new LoggedTunableNumber("Auto/IntakeDeployVoltage", -5.0);

    private RobotContainer robotContainer;

    public AutoFactory(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    Command idle() {
        return resetPose(
                new Pose2d(
                        RobotState.getInstance().getEstimatedPose().getTranslation(),
                        AllianceFlipUtil.apply(Rotation2d.kZero)));
    }

    Command nihonAuto(Trajectory<SwerveSample> traj) {
        var startPose = traj.getInitialPose(false).get();
        var time = traj.getTotalTime();

        return resetPose(startPose)
                .andThen(driveTrajectoryWithVisionState(traj, VisionState.REJECT)
                        .withDeadline(Commands.waitSeconds(time + 0.25))
                        .deadlineFor(waitUntilXCrossed(5.9, true)
                                .andThen(robotContainer.getSuperstructure().deployIntake()
                                        .alongWith(
                                                Commands.runOnce(() -> robotContainer.getIntake()
                                                        .setManualDeploy(intakeDeployVoltage.getAsDouble()))
                                                        .withDeadline(Commands
                                                                .waitSeconds(intakeDeployTime.getAsDouble()))
                                                        .finallyDo(() -> robotContainer.getIntake()
                                                                .setManualDeploy(0.0))))))
                .andThen(Commands.runOnce(() -> RobotState.getInstance().setVisionState(VisionState.ACCEPT)))
                .andThen(robotContainer.getSuperstructure().enableShooter());
        // .andThen(Commands.waitSeconds(fullShotTime.getAsDouble()))
        // .andThen(robotContainer.getSuperstructure().disableShooter());
    }

    Command vanilla(String name) {
        var traj = ChoreoUtil.getChoreoTrajectory(name);
        var startPose = traj.getInitialPose(false).get();
        // .orElse(Pose2d.kZero);

        return resetPose(startPose)
                .andThen(driveTrajectoryWithVisionState(traj, VisionState.REJECT)
                        .raceWith(Commands.waitSeconds(3.0)))
                .andThen(robotContainer.getSuperstructure().enableShooter())
                .andThen(robotContainer.getSuperstructure().deployIntake()
                        .alongWith(
                                Commands.runOnce(() -> robotContainer.getIntake()
                                        .setManualDeploy(intakeDeployVoltage.getAsDouble()))
                                        .withDeadline(Commands
                                                .waitSeconds(intakeDeployTime.getAsDouble()))
                                        .finallyDo(() -> robotContainer.getIntake()
                                                .setManualDeploy(0.0))))
                .andThen(Commands.waitSeconds(preloadShotTime.getAsDouble()))
                .andThen(robotContainer.getSuperstructure().disableShooter());
    }

    // Append
    Command mint() {
        var traj = AutoPathFactory.getMint();
        var afterShotPose = traj.getInitialPose(false).get();

        return driveToPoint(afterShotPose)
                .andThen(driveTrajectoryWithVisionState(traj, VisionState.REJECT))
                .andThen(robotContainer.getSuperstructure().shootPreset(ParamPreset.SHUB))
                .andThen(Commands.waitSeconds(refillShotTime.getAsDouble()))
                .andThen(robotContainer.getSuperstructure().disableShooter());
    }

    Command vanillaMintSwirl(String name) {
        return vanilla(name)
                .andThen(mint());
    }

    private Command jumpIntake() {
        return Commands.waitUntil(() -> robotContainer.getIntake().isStalling())
                .andThen(Commands.runOnce(() -> robotContainer.getIntake()
                        .setManualDeploy(-intakeDeployVoltage.getAsDouble()))
                        .withDeadline(Commands
                                .waitSeconds(intakeDeployTime.getAsDouble())))
                .finallyDo(() -> robotContainer.getIntake().setManualDeploy(0.0));
    }

    void autoEnd() {
        RobotState.getInstance().setVisionState(VisionState.ACCEPT);
    }

    private Command resetPose(Pose2d pose) {
        return Commands.runOnce(() -> RobotState.getInstance().resetPose(pose));
    }

    private Command driveTrajectoryWithVisionState(Trajectory<SwerveSample> traj, VisionState state) {
        return Commands.runOnce(
                () -> {
                    robotContainer.getDrive().setTrajectory(traj);
                    RobotState.getInstance().setVisionState(state);
                })
                .andThen(Commands.waitUntil(() -> robotContainer.getDrive().isTrajectoryFinished()))
                .andThen(Commands.runOnce(() -> RobotState.getInstance().setVisionState(VisionState.ACCEPT)));
    }

    private Command driveTrajectory(Trajectory<SwerveSample> traj) {
        return driveTrajectoryWithVisionState(traj, VisionState.ACCEPT);
    }

    private Command driveToPoint(Pose2d pose) {
        return driveToPoint(() -> pose);
    }

    private Command driveToPoint(Supplier<Pose2d> pose) {
        return Commands.run(() -> robotContainer.getDrive().setAutoAlignPose(pose.get()))
                .until(() -> robotContainer.getDrive().isAtAutoAlignPose());
    }

    /**
     * Returns whether robot has crossed x boundary, accounting for alliance flip
     *
     * @param xPosition         X position coordinate on blue side of field.
     * @param towardsCenterline Whether to wait until passed x coordinate towards
     *                          center line or away
     *                          from center line
     */
    private boolean xCrossed(double xPosition, boolean towardsCenterline) {
        Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
        if (AllianceFlipUtil.shouldFlip()) {
            if (towardsCenterline) {
                return robotPose.getX() < FieldConstants.fieldLength - xPosition;
            } else {
                return robotPose.getX() > FieldConstants.fieldLength - xPosition;
            }
        } else {
            if (towardsCenterline) {
                return robotPose.getX() > xPosition;
            } else {
                return robotPose.getX() < xPosition;
            }
        }
    }

    /**
     * Command that waits for x boundary to be crossed. See
     * {@link #xCrossed(double, boolean)}
     */
    private Command waitUntilXCrossed(double xPosition, boolean towardsCenterline) {
        return Commands.waitUntil(() -> xCrossed(xPosition, towardsCenterline));
    }

    /**
     * Returns whether robot has crossed y boundary, accounting for alliance flip
     *
     * @param yPosition         Y position coordinate on blue side of field.
     * @param towardsCenterline Whether to wait until passed y coordinate towards
     *                          center line or away
     *                          from center line
     */
    private boolean yCrossed(double yPosition, boolean towardsCenterline) {
        Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
        if (AllianceFlipUtil.shouldFlip()) {
            if (towardsCenterline) {
                return robotPose.getY() < FieldConstants.fieldWidth - yPosition;
            } else {
                return robotPose.getY() > FieldConstants.fieldWidth - yPosition;
            }
        } else {
            if (towardsCenterline) {
                return robotPose.getY() > yPosition;
            } else {
                return robotPose.getY() < yPosition;
            }
        }
    }

    /**
     * Command that waits for y boundary to be crossed. See
     * {@link #yCrossed(double, boolean)}
     */
    private Command waitUntilYCrossed(double yPosition, boolean towardsCenterline) {
        return Commands.waitUntil(() -> yCrossed(yPosition, towardsCenterline));
    }

    private Command printCommand(String text) {
        return Commands.runOnce(() -> System.out.println(text));
    }

    private Command notificationCommand(String notification) {
        return notificationCommand(
                new Notification(NotificationLevel.INFO, "Auto Action", notification, 3000));
    }

    private Command notificationCommand(Notification notification) { // Jank but gud enough for now
        return Commands.runOnce(() -> Elastic.sendNotification(notification));
    }
}
