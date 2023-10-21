package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;

import frc.robot.Constants.ArmConstants;

public class ArmIOSparkMax implements ArmIO {
    private final CANSparkMax motor;

    public ArmIOSparkMax() {
        motor = new CANSparkMax(ArmConstants.wristMotorID, CANSparkMax.MotorType.kBrushless);

        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor.setSmartCurrentLimit(40);
    }

    public void setArmVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.armPositionRad = motor.getEncoder().getPosition();
        inputs.armVelocityRadPerSec = motor.getEncoder().getVelocity();
        inputs.armAppliedVolts = motor.getAppliedOutput();
        inputs.armCurrentAmps = new double[] { motor.getOutputCurrent() };
        inputs.armTempCelcius = new double[] { motor.getMotorTemperature() };
    }
}
