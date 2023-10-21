package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;

import frc.robot.Constants.ArmConstants;

public class RollerIOSparkMax implements RollerIO {
    private final CANSparkMax motor;

    public RollerIOSparkMax() {
        motor = new CANSparkMax(ArmConstants.rollerMotorID, CANSparkMax.MotorType.kBrushless);

        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor.setSmartCurrentLimit(10);
    }

    public void setRollerVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        inputs.rollerAppliedVolts = motor.getAppliedOutput();
        inputs.rollerCurrentAmps = new double[] { motor.getOutputCurrent() };
        inputs.rollerTempCelcius = new double[] { motor.getMotorTemperature() };
    }
}
