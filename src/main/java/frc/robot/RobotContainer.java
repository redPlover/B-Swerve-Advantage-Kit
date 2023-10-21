// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Mode;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMAX;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.GeomUtil;
import frc.robot.util.SparkMAXBurnManager;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public static final Pose2d autoDriveTarget =
      new Pose2d(4.0, 2.0, new Rotation2d());

  // Subsystems
  private Drive drive;

  // OI objects
  private CommandJoystick leftDriverController = new CommandJoystick(0);
  private CommandJoystick rightDriverController = new CommandJoystick(1);
  private XboxController operatorController = new XboxController(2);
  private boolean isFieldRelative = true;

  // Choosers
  private final LoggedDashboardChooser<AutoRoutine> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");
  private final LoggedDashboardChooser<Double> demoLinearSpeedLimitChooser =
      new LoggedDashboardChooser<>("Linear Speed Limit");
  private final LoggedDashboardChooser<Double> demoAngularSpeedLimitChooser =
      new LoggedDashboardChooser<>("Angular Speed Limit");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Check if flash should be burned
    SparkMAXBurnManager.update();

    // Instantiate active subsystems
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case ROBOT_2023:
          drive = new Drive(new GyroIOPigeon2(), new ModuleIOSparkMAX(0),
              new ModuleIOSparkMAX(1), new ModuleIOSparkMAX(2),
              new ModuleIOSparkMAX(3));
          break;
        case ROBOT_SIMBOT:
          drive = new Drive(new GyroIO() {}, new ModuleIOSim(),
              new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim());
          break;
        default:
          break;
      }
    }

    // Instantiate missing subsystems
    drive = drive != null ? drive
        : new Drive(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {},
            new ModuleIO() {}, new ModuleIO() {});

    // Set up auto routines
    autoChooser.addDefaultOption("Do Nothing",
        new AutoRoutine(AutoPosition.ORIGIN, new InstantCommand()));
    autoChooser.addOption("Drive Characterization",
        new AutoRoutine(AutoPosition.ORIGIN,
            new FeedForwardCharacterization(drive, true,
                new FeedForwardCharacterizationData("drive"),
                drive::runCharacterizationVolts,
                drive::getCharacterizationVelocity)));
    
    // Set up choosers
    demoLinearSpeedLimitChooser.addDefaultOption("--Competition Mode--", 1.0);
    demoLinearSpeedLimitChooser.addOption("Fast Speed (70%)", 0.7);
    demoLinearSpeedLimitChooser.addOption("Medium Speed (30%)", 0.3);
    demoLinearSpeedLimitChooser.addOption("Slow Speed (15%)", 0.15);
    demoAngularSpeedLimitChooser.addDefaultOption("--Competition Mode--", 1.0);
    demoAngularSpeedLimitChooser.addOption("Fast Speed (70%)", 0.7);
    demoAngularSpeedLimitChooser.addOption("Medium Speed (30%)", 0.3);
    demoAngularSpeedLimitChooser.addOption("Slow Speed (15%)", 0.15);

    // Alert if in tuning mode
    if (Constants.tuningMode) {
      new Alert("Tuning mode active, expect decreased network performance.",
          AlertType.INFO).set(true);
    }

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Driving controls
    leftDriverController.top().onTrue(new InstantCommand(() -> {
        isFieldRelative = !isFieldRelative;
        SmartDashboard.putBoolean("FieldRelative", isFieldRelative);
    }));
    SmartDashboard.putBoolean("Field Relative", isFieldRelative);
    drive.setDefaultCommand(new DriveWithJoysticks(drive,
        () -> -leftDriverController.getX(), () -> -leftDriverController.getY(),
        () -> -rightDriverController.getX(), true));

    // Reset gyro command
    Command resetGyroCommand = new InstantCommand(() -> {
      drive.setPose(autoDriveTarget);
    }, drive).ignoringDisable(true);
    Command rumbleCommand = new StartEndCommand(
        () -> operatorController.setRumble(RumbleType.kRightRumble, 0.5),
        () -> operatorController.setRumble(RumbleType.kRightRumble, 0.0)) {
      @Override
      public boolean runsWhenDisabled() {
        return true;
      }
    }.withTimeout(0.2);
    rightDriverController.top().onTrue(resetGyroCommand).onTrue(rumbleCommand);

    // Auto drive controls
    // new Trigger(() -> driverController.getLeftTriggerAxis() > 0.5)
    // .whileTrue(new AutoDriveHard(drive));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    AutoRoutine routine = autoChooser.get();
    drive.setPose(routine.position.getPose());
    return routine.command;
  }

  private static class AutoRoutine {
    public final AutoPosition position;
    public final Command command;

    public AutoRoutine(AutoPosition position, Command command) {
      this.position = position;
      this.command = command;
    }
  }

  public static enum AutoPosition {
    ORIGIN;

    public Pose2d getPose() {
      switch (this) {
        case ORIGIN:
          return new Pose2d();
        default:
          return new Pose2d();
      }
    }
  }
}
