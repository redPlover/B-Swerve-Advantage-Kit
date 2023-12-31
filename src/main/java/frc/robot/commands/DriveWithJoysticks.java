// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;

public class DriveWithJoysticks extends CommandBase {
    private final Drive drive;
    private final DoubleSupplier xPercent;
    private final DoubleSupplier yPercent;
    private final DoubleSupplier omegaPercent;
    private final boolean fieldRelative;

    /** Creates a new DriveWithJoysticks. */
    public DriveWithJoysticks(Drive drive, DoubleSupplier xPercent, DoubleSupplier yPercent, DoubleSupplier omegaPercent, boolean fieldRelative) {
        this.drive = drive;
        this.xPercent = xPercent;
        this.yPercent = yPercent;
        this.omegaPercent = omegaPercent;
        this.fieldRelative = fieldRelative;

        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double xMPerS = processJoystickInputs(xPercent.getAsDouble(), false) * 5;
        double yMPerS = processJoystickInputs(yPercent.getAsDouble(), false) * 5;
        double omegaRadPerS = processJoystickInputs(omegaPercent.getAsDouble(), true) * 2 * Math.PI;

        //Convert to field relative speeds
        ChassisSpeeds targetSpeeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xMPerS, yMPerS, omegaRadPerS, drive.getRotation())
            : new ChassisSpeeds(xMPerS, yMPerS, omegaRadPerS);

        drive.runVelocity(targetSpeeds);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    private double processJoystickInputs(double value, boolean square) {
        double scaledValue = 0.0;
        double deadband = 0.12;
        if (Math.abs(value) > deadband) {
            scaledValue = (Math.abs(value) - deadband) / (1 - deadband);
            if (square) {
                scaledValue = Math.copySign(scaledValue * scaledValue, value);
            } else {
                scaledValue = Math.copySign(scaledValue, value);
            }
        }
        return scaledValue;
    }

}
