package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Class representing Elevator Subsystem
 * 
 * @author firearcher2012, VedNakum, Evan13019, Kunal Bareth
 */
public class Elevator extends SubsystemBase {
    private TalonFX leftMotor, rightMotor;
    private DigitalInput innerBottomLimit, outerBottomLimit;

    private double encoderValue, target;
    private double oldTarget = 0;

    private DoubleLogEntry elevatorLeftOutputCurrentLog;
    private DoubleLogEntry elevatorRightOutputCurrentLog;
    private DoubleLogEntry elevatorEncoderLog;

    boolean limitTriggered = false;
    
    TalonFXConfiguration config = new TalonFXConfiguration();
    SoftwareLimitSwitchConfigs limitConfig = new SoftwareLimitSwitchConfigs();

    MotionMagicVoltage mmVoltage = new MotionMagicVoltage(0).withSlot(0);
    Slot0Configs slot0Configs = new Slot0Configs();

    /**
     * Instantiates motors and limit switches; Set neutral modes; Assigns PID constants.
     */
    public Elevator() {
        TalonFXConfiguration leftMotorConfigs = new TalonFXConfiguration();
        TalonFXConfiguration rightMotorConfigs = new TalonFXConfiguration();

        leftMotor = new TalonFX(Constants.Elevator.LEFT_MOTOR_ID);
        rightMotor = new TalonFX(Constants.Elevator.RIGHT_MOTOR_ID);
        innerBottomLimit = new DigitalInput(Constants.Elevator.INNER_BOTTOM_LIMIT_SWITCH_SLOT);
        outerBottomLimit = new DigitalInput(Constants.Elevator.OUTER_BOTTOM_LIMIT_SWITCH_SLOT);

        leftMotorConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        rightMotorConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        leftMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // leftMotorConfigs.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = 0;
        // rightMotorConfigs.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = 0;

        leftMotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        slot0Configs.kP = Constants.Elevator.P;
        slot0Configs.kI = Constants.Elevator.I;
        slot0Configs.kD = Constants.Elevator.D;
        slot0Configs.kV = Constants.Elevator.V;
        slot0Configs.kG = Constants.Elevator.G;
        slot0Configs.GravityType = GravityTypeValue.Elevator_Static;

        MotionMagicConfigs motionMagic = new MotionMagicConfigs();
        motionMagic.MotionMagicAcceleration = Constants.Elevator.MOTION_MAGIC_ACCEL;
        motionMagic.MotionMagicCruiseVelocity = Constants.Elevator.MOTION_MAGIC_CRUISE_VELOCITY;


        leftMotor.getConfigurator().apply(leftMotorConfigs);
        rightMotor.getConfigurator().apply(rightMotorConfigs);

        rightMotor.setControl(new Follower(Constants.Elevator.LEFT_MOTOR_ID, true));
        leftMotor.getConfigurator().apply(slot0Configs);
        leftMotor.getConfigurator().apply(motionMagic);


        DataLog log = DataLogManager.getLog();

        elevatorLeftOutputCurrentLog = new DoubleLogEntry(log, "Elevator Left Output Current");
        elevatorRightOutputCurrentLog = new DoubleLogEntry(log, "Elevator Right Output Current");
        elevatorEncoderLog = new DoubleLogEntry(log, "Elevator Encoder Value");
    }

   
    /**
     * Gets encoder value; Logs data; Sets position setpoint to target; Resets encoder.
     */
    @Override
    public void periodic() {
        encoderValue = leftMotor.getPosition().getValueAsDouble();

        SmartDashboard.putNumber("Elevator Target", target);
        SmartDashboard.putNumber("Elevator Encoder Value", encoderValue);
        // SmartDashboard.putNumber("Elevator Left Motor Current", leftMotor.getSupplyCurrent().getValueAsDouble());
        // SmartDashboard.putNumber("Elevator Right Motor Current", rightMotor.getSupplyCurrent().getValueAsDouble());
        // SmartDashboard.putNumber("Elevator oldTarget", oldTarget);
        SmartDashboard.putBoolean("Limit Triggered", limitTriggered);
        SmartDashboard.putNumber("Elevator Voltage", leftMotor.getMotorVoltage().getValueAsDouble());

        
        elevatorLeftOutputCurrentLog.append(leftMotor.getSupplyCurrent().getValueAsDouble());
        elevatorRightOutputCurrentLog.append(rightMotor.getSupplyCurrent().getValueAsDouble());
        elevatorEncoderLog.append(leftMotor.getPosition().getValueAsDouble());

        leftMotor.setControl(mmVoltage.withPosition(target));

        SmartDashboard.putBoolean("Limit Inner Switch", getInnerBottomLimit());
        SmartDashboard.putBoolean("Limit Outer Switch", getOuterBottomLimit());
        SmartDashboard.putBoolean("PID Finished", isPIDFinished());

        if (getInnerBottomLimit() && getOuterBottomLimit()) {
            if (!limitTriggered) {
                resetEncoders();
                resetTarget();
                limitTriggered = true;
            } 
        } else {
            limitTriggered = false;
        }
    }

    /**
     * Sets raw power for left, right is follower 
     * @param power desired power from -1 to 1
     */
    public void setRawPower(double power) {
        leftMotor.set(power);
    }

    /**
     * Checks if PID value for left motor is within given range
     * @return whether PID is finished
     */
    public boolean isPIDFinished() {
        return (Math.abs(target - leftMotor.getPosition().getValueAsDouble()) < 0.2);
    }
    //change the value above

    /**
     * Gets motor position
     * @return left motor position as double
     */
    public double getPosition() {
        return leftMotor.getPosition().getValueAsDouble();
    }

    /**
     * Returns value of current elevator target position
     * @return current elevator target
     */
    public double getCurrentTarget() {
        return target;
    }

    /**
     * Sets the desired elevator target
     * @param target desired elevator target
     */
    public void setDesiredTarget(double target) {
        this.target = target;
    }

    /**
     * Sets enumerators for encoder positions of various Elevator States
     */
    public enum ElevatorStates{
        HOME(0),
        LV1(2.1),
        LV2(7.1),
        LV3(15.4),
        ALGAE_LOW(0),  
        ALGAE_HIGH(0),
        LV4(29.3);    

        // LV4(29.8); 
        //change the numbers above
        
        final double elevatorPosition;

        ElevatorStates(double elevatorPosition) {
            this.elevatorPosition = elevatorPosition;
        }
    }

    /**
     * Sets target as the desired Elevator state
     * @param state elevator state
     */
    public void setDesiredState(ElevatorStates state) {
        oldTarget = target;
        target = state.elevatorPosition;
    }

    /**
     * Changes target depending on joystick position
     * @param joyStickValue joystick value between 1 and -1
     */
    public void adjustSetpoint(double joyStickValue) {
        double deadbanded = (Math.abs(joyStickValue) > Constants.Controllers.DEADBAND)
                            ? joyStickValue
                            : 0;
        double newTarget = target + (deadbanded * 0.2);
        if (((newTarget >= target) && newTarget <= 30)){
            oldTarget = target;
            target = newTarget;
        } else if ((newTarget <= target)) {
            oldTarget = target;
            target = newTarget;
        }
    }

    /**
     * Returns whether the bottom limit switch is pressed
     */
    public boolean getInnerBottomLimit() {
        return !innerBottomLimit.get();
    }

    /**
     * Returns whether the bottom limit switch is pressed
     */
    public boolean getOuterBottomLimit() {
        return !outerBottomLimit.get();
    }

    /**
     * Resets encoders
     */
    public void resetEncoders() {
        leftMotor.setPosition(0);
        rightMotor.setPosition(0);
    }

    public boolean checkHeight() {
        if (target >= 3) {
            return true;
        } else{
            return false;
        }
    }

    public void resetTarget() {
        target = 0;
    }
}


