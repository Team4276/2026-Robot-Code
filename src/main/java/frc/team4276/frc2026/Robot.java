// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team4276.frc2026;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team4276.lib.VirtualSubsystem;

public class Robot extends LoggedRobot {
    private Command autonomousCommand;

    private final RobotContainer robotContainer;

    private final Timer canInitialErrorTimer = new Timer();
    private final Timer canErrorTimer = new Timer();
    private static final double canErrorTimeThreshold = 0.5; // Seconds to disable alert
    private final Alert canErrorAlert = new Alert("CAN errors detected, robot may not be controllable.",
            AlertType.kError);

    private boolean autoMessagePrinted = false;
    private Timer autoTimer = new Timer();

    public Robot() {
        Logger.recordMetadata("ProjectName", "Backup"); // Set a metadata value

        // Set up data receivers & replay source
        switch (Constants.getMode()) {
            case REAL:
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case SIM:
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
                        // be added.

        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        VirtualSubsystem.periodicAll();
        CommandScheduler.getInstance().run();

        // Print auto duration
        if (autonomousCommand != null) {
            if (!autonomousCommand.isScheduled() && !autoMessagePrinted) {
                if (DriverStation.isAutonomousEnabled()) {
                    System.out.printf("*** Auto finished in %.2f secs ***%n", autoTimer.get());
                } else {
                    System.out.printf("*** Auto cancelled in %.2f secs ***%n", autoTimer.get());
                }
                autoTimer.stop();
                autoMessagePrinted = true;
            }
        }

        // Check CAN status
        var canStatus = RobotController.getCANStatus();
        if (canStatus.transmitErrorCount > 0 || canStatus.receiveErrorCount > 0) {
            canErrorTimer.restart();
        }
        canErrorAlert.set(
                !canErrorTimer.hasElapsed(canErrorTimeThreshold)
                        && !canInitialErrorTimer.hasElapsed(canErrorTimeThreshold));
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
        autonomousCommand = robotContainer.getAutonomousCommand();
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
        robotContainer.periodic();
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
