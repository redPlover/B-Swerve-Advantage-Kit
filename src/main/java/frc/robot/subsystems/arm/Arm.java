package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    private ArmIO armIO;
    private ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();

    private IntakeIO intakeIO;
    private IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();

    private RollerIO rollerIO;
    private RollerIOInputsAutoLogged rollerInputs = new RollerIOInputsAutoLogged();

    private double wristTolerance = 1;
    private PIDController wristController = new PIDController(0, 0, 0);
    private double wristkP = 0.03;
    private double wristkI = 0.0;
    private double wristkD = 0.0;

    private final double softStop = 20.0;
    private final double hardStop = 25.0;

    private double rollerOn;

    private double shooterGoalPower;
    //private double intakeGoalPower;

    // SmartDashboard setpoint values
    private double wristIntakePosition;
    private double wristShootPosition;

    // SmartDashboard 
    private double shooterIntakePower = 0.5;
    private double shooterShootPower = -1;
    private double shooterWarmup = 0;

    //private double intakeOn, intakeOff = 0;


    public static enum Mode {
        IDLE,
        INTAKE,
        STOW,
        SHOOT_HIGH,
        SHOOT_MID,
        SHOOT_LOW,
        SPIT;
    }

    private Mode currentMode = Mode.IDLE;
    private double wristGoalPosition = 0.0;

    public Arm() {
        armIO = new ArmIOSparkMax();
        intakeIO = new IntakeIOSparkMax();
        rollerIO = new RollerIOSparkMax();

        wristController.setTolerance(wristTolerance);
        wristController.setP(wristkP);
        wristController.setI(wristkI);
        wristController.setD(wristkD);

        wristIntakePosition = ArmConstants.intakePosition;
        wristShootPosition = ArmConstants.upperShootPosition;
    }
    
    public void setMode(Mode mode) {
        currentMode = mode;
    }

    public void getSetpointFromMode (){
        SmartDashboard.putString("Mode", currentMode.toString());

        if (this.currentMode == Mode.INTAKE) {
            wristGoalPosition = ArmConstants.intakePosition;
            shooterGoalPower = shooterIntakePower;
            rollerOn = -1;
            //intakeGoalPower = intakeOn;
        } 
        else if (this.currentMode == Mode.SHOOT_HIGH) {
            wristGoalPosition = ArmConstants.upperShootPosition;
            shooterGoalPower = ArmConstants.highShootVoltage;
            rollerOn = 1;
            //intakeGoalPower = intakeOff;
        } 
        else if (this.currentMode == Mode.SHOOT_MID) {
            wristGoalPosition = ArmConstants.upperShootPosition;
            shooterGoalPower = ArmConstants.midShootVoltage;
            rollerOn = 1;
            //intakeGoalPower = intakeOff;
        } 
        else if (this.currentMode == Mode.SHOOT_LOW) {
            wristGoalPosition = ArmConstants.upperShootPosition;
            shooterGoalPower = ArmConstants.lowShootVoltage;
            rollerOn = 1;
            //intakeGoalPower = intakeOff;
        } 
        else if (this.currentMode == Mode.STOW) {
            wristGoalPosition = ArmConstants.stowShootPosition;
            shooterGoalPower = 0.0;
            rollerOn = 0;
            //intakeGoalPower = intakeOff;
        }
        else if (this.currentMode == Mode.SPIT){
            wristGoalPosition = ArmConstants.intakePosition;
            if (wristFinished()){
                shooterGoalPower = -ArmConstants.spitVoltage;
                rollerOn = 1;
            }
            else{
                shooterGoalPower = 0;
                rollerOn = 0;
                
            }
        }
        else {
            //IDLE MODE
            wristGoalPosition = ArmConstants.stowShootPosition;
        }
    }

    public double getWristPosition() {
        return armInputs.armPositionRad;
    }

    private void setWristMotorPower(double percent) {
        armIO.setArmVoltage(percent * 12);
    }

    public double getWristMotorAmps() {
        return armInputs.armCurrentAmps[0];
    }

    private void checkWristAmps() {
        if (getWristMotorAmps() > hardStop) {
            setWristMotorPower(0);
            setMode(Mode.IDLE);
        }
    }


    public void wristControl() {
        double output = wristController.calculate(getWristPosition(), wristGoalPosition);
        setWristMotorPower(output);
        // checkWristAmps();
    }

    public boolean wristFinished() {
        return Math.abs(getWristPosition()-wristGoalPosition) < wristTolerance;
    }

    public void setRollerPower(double percent) {
        rollerIO.setRollerVoltage(percent*0.5);
    }
   
    public void setIntakeMotorPower(double percent) {
        intakeIO.setIntakeVoltage(percent*12);
    }

    public double getIntakeMotorAmps() {
        return intakeInputs.intakeCurrentAmps[0];
    } 

    private void checkIntakeAmps() {
        if (getIntakeMotorAmps() > hardStop) {
            setIntakeMotorPower(0);
        }
    }

    @Override
    public void periodic() {
        armIO.updateInputs(armInputs);

        getSetpointFromMode();
        setIntakeMotorPower(shooterGoalPower);
        setRollerPower(rollerOn);
        wristControl();

        wristIntakePosition = SmartDashboard.getNumber("wrist intake position", 0);
        wristShootPosition = SmartDashboard.getNumber("wrist shoot position", 0.5);

        shooterIntakePower = SmartDashboard.getNumber("intake power", 0.5);
        shooterShootPower = SmartDashboard.getNumber("shoot power", -1);

        wristkP = SmartDashboard.getNumber("wrist kP", 0.03);
        wristkI = SmartDashboard.getNumber("wrist kI", 0.0);
        wristkD = SmartDashboard.getNumber("wrist kD", 0.0);

        wristController.setPID(wristkP, wristkI, wristkD);

        SmartDashboard.putNumber("wrist intake position", wristIntakePosition);
        SmartDashboard.putNumber("wrist shoot position", wristShootPosition);

        SmartDashboard.putNumber("intake power", shooterIntakePower);
        SmartDashboard.putNumber("shoot power", shooterShootPower);

        SmartDashboard.putNumber("wrist kP", wristkP);
        SmartDashboard.putNumber("wrist kI", wristkI);
        SmartDashboard.putNumber("wrist kD", wristkD);
        // checkIntakeAmps();
    }
}
