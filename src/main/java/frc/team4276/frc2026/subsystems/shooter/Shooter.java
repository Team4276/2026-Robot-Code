package frc.team4276.frc2026.subsystems.shooter;

import static frc.team4276.frc2026.subsystems.shooter.ShooterConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2026.subsystems.shooter.ShotCalculator.ShootingParameters;
import frc.team4276.frc2026.subsystems.shooter.flywheel.FlywheelIO;
import frc.team4276.frc2026.subsystems.shooter.flywheel.FlywheelIOInputsAutoLogged;

public class Shooter extends SubsystemBase {
    private FlywheelIO flywheelIO;
    private FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();

    public enum WantedState {
        IDLE,
        SHOOT,
        FERRY
    }

    public enum SystemState {
        IDLING,
        SHOOTING,
        FERRYING
    }

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLING;

    private ShootingParameters requestedShooterState = new ShootingParameters(
                        true,
                        Rotation2d.kZero,
                        0.0,
                        0.0,
                        0.0,
                        0.0);

    public Shooter(FlywheelIO flywheelIO) {
        this.flywheelIO = flywheelIO;
    }

    @Override
    public void periodic() {
        flywheelIO.updateInputs(flywheelInputs);
        Logger.processInputs("Flywheel", flywheelInputs);

        systemState = handleStateTransition();
        applyState();

        Logger.recordOutput("Shooter/SystemState", systemState);
        Logger.recordOutput("Shooter/DesiredState", wantedState);
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case IDLE -> SystemState.IDLING;
            case SHOOT -> SystemState.SHOOTING;
            case FERRY -> SystemState.FERRYING;
        };
    }

    private void applyState() {
        switch (systemState) {
            case IDLING:
                requestedShooterState = new ShootingParameters(
                        true,
                        Rotation2d.kZero,
                        0.0,
                        0.0,
                        0.0,
                        0.0);

                break;
            case SHOOTING:
                requestedShooterState = ShotCalculator.getInstance().getHubParameters();

                break;
            case FERRYING:
                requestedShooterState = new ShootingParameters(
                        true,
                        Rotation2d.kZero,
                        0.0,
                        0.0,
                        0.0,
                        0.0);

                break;
        }

        flywheelIO.setRpm(requestedShooterState.flywheelSpeed());
    }

    public void setWantedState(WantedState state) {
        wantedState = state;
    }

    public boolean atGoal() {
        return MathUtil.isNear(requestedShooterState.flywheelSpeed(), flywheelInputs.velocity, tolerance);
    }
}
