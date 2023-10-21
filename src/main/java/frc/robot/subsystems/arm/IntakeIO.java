package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double intakeAppliedVolts = 0.0;
        public double[] intakeCurrentAmps = new double[] {};
        public double[] intakeTempCelcius = new double[] {};
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setIntakeVoltage(double volts) {}
}
