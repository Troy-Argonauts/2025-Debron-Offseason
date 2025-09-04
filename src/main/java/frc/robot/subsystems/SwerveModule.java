package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
// Necessary imports for our SwerveModule
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

import com.ctre.phoenix6.configs.*;
import static frc.robot.Constants.Swerve.*;
import static frc.robot.Constants.SwerveModule.*;

/**
 * Constructor for SwerveModule. Sets up motors, encoders, and configurations.
 *
 * @param driveMotorID        ID for the drive motor
 * @param turnMotorID         ID for the turn motor
 * @param turnEncoderID       ID for the turn encoder
 * @param canbusName          CAN bus name
 * @param chassisAngularOffset Angular offset for chassis
 * @param driveInverted       Whether the drive motor is inverted
*/
public class SwerveModule extends SubsystemBase{
    private TalonFX driveMotor, turnMotor;
    public double driveValue;
    private CANcoder turnEncoder;
    public double turnEncoderValue = 0;
    public double turnEncoderRotation = 0;

    private TalonFXConfiguration driveTalonConfig = new TalonFXConfiguration();
    private TalonFXConfiguration turnTalonConfig = new TalonFXConfiguration();
    public SwerveModuleState globalDesiredState = new SwerveModuleState();

    MotionMagicVoltage turnMMVoltage = new MotionMagicVoltage(0).withSlot(1);
    MotionMagicVelocityVoltage driveMMVoltage = new MotionMagicVelocityVoltage(0).withSlot(0);
    
    
    MotionMagicConfigs driveMMConfig = driveTalonConfig.MotionMagic;
    MotionMagicConfigs turnMMConfig = turnTalonConfig.MotionMagic;

    private Slot0Configs driveSlotConfig = driveTalonConfig.Slot0;
    private Slot1Configs turnSlotConfig = turnTalonConfig.Slot1;
    
    public SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());
    private double chassisAngularOffset;

    public SwerveModule(int driveMotorID, int turnMotorID, int turnEncoderID, String canbusName, double chassisAngularOffset, boolean driveInverted){
        driveMotor = new TalonFX(driveMotorID, canbusName);
        turnMotor = new TalonFX(turnMotorID, canbusName);
        driveValue = 0;
        turnEncoder = new CANcoder(turnEncoderID, canbusName);

        CANcoderConfiguration encoderConfigs = new CANcoderConfiguration();
        encoderConfigs.MagnetSensor.MagnetOffset = chassisAngularOffset; // make sure this is in rotations
        turnEncoder.getConfigurator().apply(encoderConfigs);

        TalonFXConfiguration turnSlotConfigs = new TalonFXConfiguration();
        turnTalonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        turnTalonConfig.ClosedLoopGeneral.ContinuousWrap = true;
        turnTalonConfig.Feedback.FeedbackRemoteSensorID = turnEncoder.getDeviceID();
        turnTalonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        turnTalonConfig.CurrentLimits.SupplyCurrentLimit = TURNING_MOTOR_CURRENT_LIMIT;
        turnTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnSlotConfig.kP = TURN_P;
        turnSlotConfig.kI = TURN_I;
        turnSlotConfig.kD = TURN_D;
        turnSlotConfig.kS = TURN_S;
        turnSlotConfig.kV = TURN_V;
        turnSlotConfig.kA = TURN_A;
        turnMotor.getConfigurator().apply(turnSlotConfigs);

        turnMMConfig.MotionMagicCruiseVelocity = TURN_MM_CRUISE_VELOCITY;
        turnMMConfig.MotionMagicAcceleration = TURN_MM_ACCELERATION;
        turnMMConfig.MotionMagicJerk = TURN_MM_JERK;

        turnMotor.getConfigurator().apply(turnTalonConfig);

        driveSlotConfig.kP = DRIVE_P;
        driveSlotConfig.kI = DRIVE_I;
        driveSlotConfig.kD = DRIVE_D;
        driveSlotConfig.kS = DRIVE_S;
        driveSlotConfig.kV = DRIVE_V;

        if (driveInverted){
            driveTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            driveTalonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            driveTalonConfig.CurrentLimits.SupplyCurrentLimit = DRIVING_MOTOR_CURRENT_LIMIT;
            driveMMConfig.MotionMagicAcceleration = DRIVE_MM_ACCELERATION;
            driveMMConfig.MotionMagicJerk = DRIVE_MM_JERK;
        
            driveMotor.getConfigurator().apply(driveTalonConfig);
        } else {
            driveTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            driveTalonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            driveTalonConfig.CurrentLimits.SupplyCurrentLimit = DRIVING_MOTOR_CURRENT_LIMIT;
            driveMMConfig.MotionMagicAcceleration = DRIVE_MM_ACCELERATION;
            driveMMConfig.MotionMagicJerk = DRIVE_MM_JERK;
            driveMotor.getConfigurator().apply(driveTalonConfig);
            
        }

        turnMotor.getConfigurator().apply(turnSlotConfig);
        driveMotor.getConfigurator().apply(driveSlotConfig);



        //RESET ENCODERS
        resetDriveEncoder();

        this.chassisAngularOffset = chassisAngularOffset;
    }

    /**
     * Updates the driveEncoder and turnEncoder periodically.
     */
    @Override
    public void periodic() {
        driveValue = driveMotor.getPosition().getValueAsDouble();
        turnEncoderValue = getAngle();
        turnEncoderRotation = turnEncoder.getPosition().getValueAsDouble();


        turnMotor.setControl(turnMMVoltage.withPosition(globalDesiredState.angle.getDegrees() / 360));
        // driveMotor.setControl(mmRequest.withVelocity(globalDesiredState.speedMetersPerSecond * Constants.Swerve.DRIVING_MOTOR_REDUCTION / WHEEL_CIRCUMFERENCE_METERS));
        driveMotor.setControl(driveMMVoltage.withVelocity(globalDesiredState.speedMetersPerSecond * Constants.Swerve.DRIVING_MOTOR_REDUCTION / WHEEL_CIRCUMFERENCE_METERS));

        SmartDashboard.putNumber("Corrected Rotation", globalDesiredState.angle.getDegrees() / 360);
        SmartDashboard.putNumber("Corrected Speed", globalDesiredState.speedMetersPerSecond);
        SmartDashboard.putNumber("Desired State Rotation", desiredState.angle.getDegrees());
        SmartDashboard.putNumber("Desired State Speed", desiredState.speedMetersPerSecond);

    }

   /**
   * Returns the current state of the module.
   * 
   * @return The current state of the module.
   */
    public SwerveModuleState getState() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModuleState(getVelocity(),
            new Rotation2d(turnEncoderValue - chassisAngularOffset));
    }

   /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
    public SwerveModulePosition getPosition() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModulePosition(
            driveValue,
            new Rotation2d(turnEncoderValue - chassisAngularOffset));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        SmartDashboard.putNumber("Desired State Angle2",desiredState.angle.getDegrees());
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromDegrees(chassisAngularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        correctedDesiredState.optimize(Rotation2d.fromDegrees(turnEncoderValue));

        // Command driving and turning motors towards their respective setpoints (velocity and position).
        // turnMotor.setControl(positionVoltage.withPosition(correctedDesiredState.angle.getDegrees() / 360));
        // driveMotor.setControl(mmRequest.withVelocity(correctedDesiredState.speedMetersPerSecond / WHEEL_CIRCUMFERENCE_METERS));

        

        this.desiredState = desiredState;
        globalDesiredState = correctedDesiredState;
    }

    // /**
    // * Configures feedback for the turn encoder.
    // */
    // public void configureFeedback() {
    //     CANcoderConfiguration encoderConfigs = new CANcoderConfiguration();
    //     encoderConfigs.MagnetSensor.MagnetOffset = chassisAngularOffset; // make sure this is in rotations

    //     TalonFXConfiguration turnSlotConfigs = new TalonFXConfiguration();
    //     turnSlotConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    //     turnSlotConfigs.ClosedLoopGeneral.ContinuousWrap = true;
    //     turnSlotConfigs.Feedback.FeedbackRemoteSensorID = turnEncoder.getDeviceID();
    //     turnSlotConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    // }


    /** Zeroes the driveMotor encoder on the SwerveModule */
    public void resetDriveEncoder(){
        driveMotor.setPosition(0);
    }

    /** Zeroes the turnMotor encoder on the SwerveModule */
    public void resetTurnEncoder(){
        turnEncoder.setPosition(0);
    }

    /**
     * Returns the velocity of the drive motor in meters per second.
     * @return velocity of the drive motor in meters per second
     */
    public double getVelocity(){
        return driveMotor.getVelocity().getValueAsDouble() * WHEEL_CIRCUMFERENCE_METERS / Constants.Swerve.DRIVING_MOTOR_REDUCTION ;
    }

    /** Returns the angle of the drive motor 
     * 360 accounts for the turnEncoder output from 0 to 1 to conver it to degrees
    */
    public double getAngle(){
        return turnEncoder.getPosition().getValueAsDouble() * 360;
    }

    public double getFrontPositionMeters(){
        return driveMotor.getPosition().getValueAsDouble() * WHEEL_CIRCUMFERENCE_METERS / Constants.Swerve.DRIVING_MOTOR_REDUCTION;
    }
}
