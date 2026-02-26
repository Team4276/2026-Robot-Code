package frc.team4276.frc2026.subsystems.intake;

import frc.team4276.lib.dashboard.LoggedTunableNumber;

public class IntakeConstants {
    public static final LoggedTunableNumber deployPosition = new LoggedTunableNumber("Intake/DeployPosition", 0.0);
    public static final LoggedTunableNumber retractPosition = new LoggedTunableNumber("Intake/RetractPosition", 0.0);

    public static final LoggedTunableNumber idleVolts = new LoggedTunableNumber("Intake/IdleVolts", -2.0);
    public static final LoggedTunableNumber intakeVolts = new LoggedTunableNumber("Intake/IntakeVolts", 12.0);
    public static final LoggedTunableNumber exhaustVolts = new LoggedTunableNumber("Intake/ExhaustVolts", -6.0);
}
