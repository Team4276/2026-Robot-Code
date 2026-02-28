// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team4276.frc2026;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team4276.frc2026.shooter.ShooterConstants.ParamPreset;
import frc.team4276.frc2026.subsystems.Superstructure;
import frc.team4276.frc2026.subsystems.drive.Drive;
import frc.team4276.frc2026.subsystems.drive.Gyro.GyroIO;
import frc.team4276.frc2026.subsystems.drive.Gyro.GyroIOPigeon2;
import frc.team4276.frc2026.subsystems.drive.Module.ModuleIO;
import frc.team4276.frc2026.subsystems.drive.Module.ModuleIOKreo;
import frc.team4276.frc2026.subsystems.drive.Module.ModuleIOSim;
import frc.team4276.frc2026.subsystems.feeder.Feeder;
import frc.team4276.frc2026.subsystems.feeder.FeederIO;
import frc.team4276.frc2026.subsystems.feeder.FeederIOSpark;
import frc.team4276.frc2026.subsystems.flywheel.Flywheel;
import frc.team4276.frc2026.subsystems.flywheel.FlywheelIO;
import frc.team4276.frc2026.subsystems.flywheel.FlywheelIOSpark;
import frc.team4276.frc2026.subsystems.intake.Intake;
import frc.team4276.frc2026.subsystems.intake.IntakeDeployIO;
import frc.team4276.frc2026.subsystems.intake.IntakeDeployIOSpark;
import frc.team4276.frc2026.subsystems.intake.IntakeRollerIO;
import frc.team4276.frc2026.subsystems.intake.IntakeRollerIOSpark;
import frc.team4276.frc2026.subsystems.vision.Vision;
import frc.team4276.frc2026.subsystems.vision.VisionIO;
import frc.team4276.frc2026.subsystems.vision.VisionIOPhotonVision;
import frc.team4276.lib.geometry.AllianceFlipUtil;
import frc.team4276.lib.hid.CowsController;
import frc.team4276.lib.hid.ViXController;

public class RobotContainer {
    private Drive drive;
    private Intake intake;
    private Feeder feeder;
    private Flywheel flywheel;
    private Vision vision;

    private final Superstructure superstructure;

    private final ViXController driver = new ViXController(Ports.DRIVER_CONTROLLER);
    private final CowsController demoController = new CowsController(Ports.DEMO_CONTROLLER_LEFT,
            Ports.DEMO_CONTROLLER_RIGHT);

    public RobotContainer() {
        if (Constants.getMode() != Constants.Mode.REPLAY) {
            switch (Constants.getType()) {
                case COMPBOT -> {
                    // Real robot, instantiate hardware IO implementations
                    drive = new Drive(
                            Constants.isDemo ? demoController : driver,
                            new GyroIOPigeon2(),
                            new ModuleIOKreo(0),
                            new ModuleIOKreo(1),
                            new ModuleIOKreo(2),
                            new ModuleIOKreo(3));
                    intake = new Intake(new IntakeDeployIOSpark(), new IntakeRollerIOSpark());
                    feeder = new Feeder(new FeederIOSpark());
                    flywheel = new Flywheel(new FlywheelIOSpark());
                    vision = new Vision(RobotState.getInstance()::addVisionMeasurement, new VisionIOPhotonVision(0));
                }

                case SIMBOT -> {
                    // Sim robot, instantiate physics sim IO implementations
                    drive = new Drive(
                            Constants.isDemo ? demoController : driver,
                            new GyroIO() {
                            },
                            new ModuleIOSim(),
                            new ModuleIOSim(),
                            new ModuleIOSim(),
                            new ModuleIOSim());
                    intake = new Intake(new IntakeDeployIO() {
                    }, new IntakeRollerIO() {
                    });
                    feeder = new Feeder(new FeederIO() {
                    });
                    flywheel = new Flywheel(new FlywheelIO() {
                    });
                    vision = new Vision(RobotState.getInstance()::addVisionMeasurement);
                }
            }
        }

        // No-op implmentations for replay
        if (drive == null) {
            drive = new Drive(
                    Constants.isDemo ? demoController : driver,
                    new GyroIO() {
                    },
                    new ModuleIO() {
                    },
                    new ModuleIO() {
                    },
                    new ModuleIO() {
                    },
                    new ModuleIO() {
                    });
        }

        if (intake == null) {
            intake = new Intake(new IntakeDeployIO() {
            }, new IntakeRollerIO() {
            });
        }

        if (feeder == null) {
            feeder = new Feeder(new FeederIO() {
            });
        }

        if (flywheel == null) {
            flywheel = new Flywheel(new FlywheelIO() {
            });
        }

        if (vision == null) {
            vision = new Vision(RobotState.getInstance()::addVisionMeasurement, new VisionIO() {
            });
        }

        superstructure = new Superstructure(drive, intake, feeder, flywheel, vision, driver);

        configureBindings();

        DriverStation.silenceJoystickConnectionWarning(true);
    }

    private void configureBindings() {
        driver
                .start()
                .onTrue(
                        Commands.runOnce(
                                () -> RobotState.getInstance()
                                        .resetPose(
                                                new Pose2d(
                                                        RobotState.getInstance().getEstimatedPose().getTranslation(),
                                                        AllianceFlipUtil.apply(Rotation2d.kZero))))
                                .ignoringDisable(true));

        driver
                .rightTrigger()
                .whileTrue(superstructure.enableShooter());

        driver
                .rightBumper()
                .onTrue(superstructure.disableShooter());

        driver
                .leftTrigger()
                .onTrue(superstructure.deployIntake());

        driver
                .leftBumper()
                .onTrue(superstructure.retractIntake());

        driver
                .y()
                .onTrue(superstructure.shootPreset(ParamPreset.SHUB));

        driver
                .a()
                .onTrue(superstructure.shootPreset(ParamPreset.SHOWER));

        driver
                .x()
                .onTrue(superstructure.shootPreset(ParamPreset.SHERRY));

        driver
                .b()
                .onTrue(superstructure.turtle());

        driver
                .povUp()
                .onTrue(Commands.runOnce(() -> superstructure.setIsFirstActive(true))
                        .ignoringDisable(true));

        driver
                .povDown()
                .onTrue(Commands.runOnce(() -> superstructure.setIsFirstActive(false))
                        .ignoringDisable(true));

        driver
                .povRight()
                .onTrue(Commands.runOnce(() -> superstructure.setOverrideActive(false))
                        .ignoringDisable(true));

        driver
                .povLeft()
                .onTrue(Commands.runOnce(() -> superstructure.setOverrideActive(true))
                        .ignoringDisable(true));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
