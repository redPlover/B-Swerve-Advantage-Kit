package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;

import frc.robot.Constants.ArmConstants;

public class IntakeIOSparkMax implements IntakeIO {
    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;

    public IntakeIOSparkMax() {
        leftMotor = new CANSparkMax(ArmConstants.leftIntakeMotorID, CANSparkMax.MotorType.kBrushless);
        rightMotor = new CANSparkMax(ArmConstants.rightIntakeMotorID, CANSparkMax.MotorType.kBrushless);

        leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        leftMotor.setSmartCurrentLimit(40);
        rightMotor.setSmartCurrentLimit(40);

        rightMotor.follow(leftMotor, true);
    }

    public void setIntakeVoltage(double volts) {
        leftMotor.setVoltage(volts);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeAppliedVolts = leftMotor.getAppliedOutput();
        inputs.intakeCurrentAmps = new double[] { leftMotor.getOutputCurrent(), rightMotor.getOutputCurrent() };
        inputs.intakeTempCelcius = new double[] { leftMotor.getMotorTemperature(), rightMotor.getMotorTemperature() };
    }
}
