package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
    @AutoLog
    public static class RollerIOInputs {
        public double rollerAppliedVolts = 0.0;
        public double[] rollerCurrentAmps = new double[] {};
        public double[] rollerTempCelcius = new double[] {};
    }

    public default void updateInputs(RollerIOInputs inputs) {}

    public default void setRollerVoltage(double volts) {}
}
