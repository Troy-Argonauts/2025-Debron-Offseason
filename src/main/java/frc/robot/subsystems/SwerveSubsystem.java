package frc.robot.subsystems;

import static frc.robot.Constants.Swerve.*;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import frc.robot.Constants;
import frc.robot.Constants.Swerve;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {

    private MotionMagicVelocityVoltage mmVelocityRequest = new MotionMagicVelocityVoltage(0);

    // Create SwerveModules
    private final SwerveModule frontLeftModule = new SwerveModule(
        FRONT_LEFT_DRIVING_CAN_ID,
        FRONT_LEFT_TURNING_CAN_ID,
        FRONT_LEFT_CAN_ENCODER,
        CANBUS_NAME,
        FRONT_LEFT_CHASSIS_ANGULAR_OFFSET,
        FRONT_LEFT_DRIVE_INVERTED);

    private final SwerveModule frontRightModule = new SwerveModule(
        FRONT_RIGHT_DRIVING_CAN_ID,
        FRONT_RIGHT_TURNING_CAN_ID,
        FRONT_RIGHT_CAN_ENCODER,
        CANBUS_NAME,
        FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET,
        FRONT_RIGHT_DRIVE_INVERTED);

    private final SwerveModule backLeftModule = new SwerveModule(
        BACK_LEFT_DRIVING_CAN_ID,
        BACK_LEFT_TURNING_CAN_ID,
        BACK_LEFT_CAN_ENCODER,
        CANBUS_NAME,
        BACK_LEFT_CHASSIS_ANGULAR_OFFSET,
        BACK_LEFT_DRIVE_INVERTED);
        
    private final SwerveModule backRightModule = new SwerveModule(
        BACK_RIGHT_DRIVING_CAN_ID,
        BACK_RIGHT_TURNING_CAN_ID,
        BACK_RIGHT_CAN_ENCODER,
        CANBUS_NAME,
        BACK_RIGHT_CHASSIS_ANGULAR_OFFSET,
        BACK_RIGHT_DRIVE_INVERTED);


    public boolean xState = false;
    public boolean zState = false;

    public boolean slow = false;

    // The gyro sensor
    public final Pigeon2 gyro = new Pigeon2(PIGEON_CAN_ID, CANBUS_NAME);


    // Slew rate filter variables for controlling lateral acceleration
    private double currentRotation = 0.0;
    private double currentTranslationDir = 0.0;
    private double currentTranslationMag = 0.0;

    private SlewRateLimiter magLimiter = new SlewRateLimiter(Swerve.MAGNITUDE_SLEW_RATE);
    private SlewRateLimiter rotLimiter = new SlewRateLimiter(Swerve.ROTATIONAL_SLEW_RATE);
    private double prevTime = WPIUtilJNI.now() * 1e-6;

    private ModuleConfig moduleConfig;
    private RobotConfig robotConfig;

    public PIDController pidX = new PIDController(0, 0, 0);
    public PIDController pidY = new PIDController(0, 0, 0);
    public PIDController pidRot = new PIDController(0, 0, 0);


    // Odometry class for tracking robot pose
    SwerveDriveOdometry odometry = new SwerveDriveOdometry(
        Swerve.DRIVE_KINEMATICS,
        Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()),
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition()
    });

    public SwerveSubsystem() {
        moduleConfig = new ModuleConfig(
            Constants.Swerve.WHEEL_DIAMETER_METERS / 2, 
            Constants.Swerve.MAX_SPEED_METERS_PER_SECOND * 0.85, 
            Constants.PathPlanner.WHEEL_COF,
            DCMotor.getKrakenX60(1), 
            Constants.SwerveModule.DRIVING_MOTOR_CURRENT_LIMIT, 1);
        robotConfig = new RobotConfig(
            Constants.PathPlanner.ROBOT_MASS, 
            Constants.PathPlanner.MOMENT_OF_INERTIA, 
            moduleConfig, 
            moduleOffsets);

        // robotConfig = RobotConfig.fromGUISettings();
        // try {
        //     robotConfig = RobotConfig.fromGUISettings();
        // } catch (Exception e) {
        //     e.printStackTrace();
        // }

            AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> pathPlannerDrive(speeds, true), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(7.75, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(7.75, 0.0, 0.0) // Rotation PID constants
            ),
            robotConfig, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
    
    }

    /**
     * Steps a value (angle) towards a target (angle) taking the shortest path with a specified step size.
     * @param _current The current or starting angle (in radians).  Can lie outside the 0 to 2*PI range.
     * @param _target The target angle (in radians) the algorithm will step towards.  Can lie outside the 0 to 2*PI range.
     * @param _stepsize The maximum step size that can be taken (in radians).
     * @return The new angle (in radians) for {@code _current} after performing the specified step towards the specified target.
     * This value will always lie in the range 0 to 2*PI (exclusive).
     */
    public static double stepTowardsCircular(double _current, double _target, double _stepsize) {
        _current = wrapAngle(_current);
        _target = wrapAngle(_target);

        double stepDirection = Math.signum(_target - _current);
        double difference = Math.abs(_current - _target);
        
        if (difference <= _stepsize) { 
            return _target; 
        } else if (difference > Math.PI) { //does the system need to wrap over eventually?
            //handle the special case where you can reach the target in one step while also wrapping
            if (_current + 2*Math.PI - _target < _stepsize || _target + 2*Math.PI - _current < _stepsize) { 
                return _target; 
            } else { 
                return wrapAngle(_current - stepDirection * _stepsize); /*this will handle wrapping gracefully*/ 
            }
        } else { 
            return _current + stepDirection * _stepsize; 
        }
    }

    /**
     * Finds the (unsigned) minimum difference between two angles including calculating across 0.
     * @param _angleA An angle (in radians).
     * @param _angleB An angle (in radians).
     * @return The (unsigned) minimum difference between the two angles (in radians).
     */
    public static double angleDifference(double _angleA, double _angleB) {
        double difference = Math.abs(_angleA - _angleB);
        return difference > Math.PI? (2 * Math.PI) - difference : difference;
    }

    /**
     * Wraps an angle until it lies within the range from 0 to 2*PI (exclusive).
     * @param _angle The angle (in radians) to wrap.  Can be positive or negative and can lie multiple wraps outside the output range.
     * @return An angle (in radians) from 0 and 2*PI (exclusive).
     */
    public static double wrapAngle(double _angle) {
        double twoPi = 2*Math.PI;

        if (_angle == twoPi) { // Handle this case separately to avoid floating point errors with the floor after the division in the case below
            return 0.0;
        } else if (_angle > twoPi) {
            return _angle - twoPi*Math.floor(_angle / twoPi);
        } else if (_angle < 0.0) {
            return _angle + twoPi*(Math.floor((-_angle) / twoPi)+1);
        } else {
            return _angle;
        }
    }


    /**
     * Updates the odometry to the current positions of all the wheels and gyro, outputs the turn and drive encoder values for 
     * each swerve module, and also outputs the gyro angle.
     */
    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        odometry.update(
            Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()),
            new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
            });

        SmartDashboard.putNumber("Gyro Angle", gyro.getYaw().getValueAsDouble());
        
        SmartDashboard.putNumber("FL Drive Encoder", frontLeftModule.driveValue);
        SmartDashboard.putNumber("FR Drive Encoder", frontRightModule.driveValue);
        SmartDashboard.putNumber("BL Drive Encoder", backLeftModule.driveValue);
        SmartDashboard.putNumber("BR Drive Encoder", backRightModule.driveValue);


        SmartDashboard.putNumber("FL Drive Encoder Velocity", frontLeftModule.getVelocity());
        SmartDashboard.putNumber("FR Drive Encoder Velocity", frontRightModule.getVelocity());
        SmartDashboard.putNumber("BL Drive Encoder Velocity", backLeftModule.getVelocity());
        SmartDashboard.putNumber("BR Drive Encoder Velocity", backRightModule.getVelocity());


        SmartDashboard.putNumber("FL Desired State Velocity", frontLeftModule.desiredState.speedMetersPerSecond);
        SmartDashboard.putNumber("FR Desired State Velocity", frontRightModule.desiredState.speedMetersPerSecond);
        SmartDashboard.putNumber("BL Desired State Velocity", backLeftModule.desiredState.speedMetersPerSecond);
        SmartDashboard.putNumber("BR Desired State Velocity", backRightModule.desiredState.speedMetersPerSecond);

        SmartDashboard.putNumber("FL Turn Angle", frontLeftModule.getAngle());
        SmartDashboard.putNumber("FR Turn Angle", frontRightModule.getAngle());
        SmartDashboard.putNumber("BL Turn Angle", backLeftModule.getAngle());
        SmartDashboard.putNumber("BR Turn Angle", backRightModule.getAngle());
    }



    /**
   * Returns the currently-estimated pose of the robot.
   * @return The pose.
   */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }


    /**
   * Resets the odometry to the specified pose.
   * @param pose The pose to which to set the odometry.
   */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
            Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()),
            new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
            },
            pose);
    }


   /**
   * Method to drive the robot using joystick info.
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
        
        double xSpeedCommanded;
        double ySpeedCommanded;
        double xSpeedCorrected;
        double ySpeedCorrected;
        double rotCorrected;

        if(slow){
            xSpeedCorrected = xSpeed *0.11;
            ySpeedCorrected = ySpeed *0.11;
            rotCorrected = rot *0.11;
        } else {
            xSpeedCorrected = xSpeed;
            ySpeedCorrected = ySpeed;
            rotCorrected = rot;
        }

        if (xState && !zState){
            frontLeftModule.setDesiredState(new SwerveModuleState(4.5, Rotation2d.fromDegrees(0)));
            frontRightModule.setDesiredState(new SwerveModuleState(4.5, Rotation2d.fromDegrees(0)));
            backLeftModule.setDesiredState(new SwerveModuleState(4.5, Rotation2d.fromDegrees(0)));
            backRightModule.setDesiredState(new SwerveModuleState(4.5, Rotation2d.fromDegrees(0)));
        }
        else if (!xState && zState){
            frontLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
            frontRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
            backLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
            backRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
        }
         else {
            if (rateLimit) {
                // Convert XY to polar for rate limiting
                double inputTranslationDir = Math.atan2(ySpeedCorrected, xSpeedCorrected);
                double inputTranslationMag = Math.sqrt(Math.pow(xSpeedCorrected, 2) + Math.pow(ySpeedCorrected, 2));

                // Calculate the direction slew rate based on an estimate of the lateral acceleration
                double directionSlewRate;
                if (currentTranslationMag != 0.0) {
                    directionSlewRate = Math.abs(Swerve.DIRECTION_SLEW_RATE / currentTranslationMag);
                } else {
                    directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
                }
                
                double currentTime = WPIUtilJNI.now() * 1e-6;
                double elapsedTime = currentTime - prevTime;
                double angleDif = angleDifference(inputTranslationDir, currentTranslationDir);
                if (angleDif < 0.45*Math.PI){
                    currentTranslationDir = stepTowardsCircular(currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
                    currentTranslationMag = magLimiter.calculate(inputTranslationMag);
                } else if (angleDif > 0.85*Math.PI) {
                    if (currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
                        // keep currentTranslationDir unchanged
                        currentTranslationMag = magLimiter.calculate(0.0);
                    } else {
                        currentTranslationDir = wrapAngle(currentTranslationDir + Math.PI);
                        currentTranslationMag = magLimiter.calculate(inputTranslationMag);
                    }
                } else {
                    currentTranslationDir = stepTowardsCircular(currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
                    currentTranslationMag = magLimiter.calculate(0.0);
                }
                prevTime = currentTime;
                xSpeedCommanded = currentTranslationMag * Math.cos(currentTranslationDir);
                ySpeedCommanded = currentTranslationMag * Math.sin(currentTranslationDir);
                currentRotation = rotLimiter.calculate(rotCorrected);


            } else {
                xSpeedCommanded = xSpeedCorrected;
                ySpeedCommanded = ySpeedCorrected;
                currentRotation = rotCorrected;
            }
        

            // Convert the commanded speeds into the correct units for the drivetrain
            double xSpeedDelivered = xSpeedCommanded * Swerve.MAX_SPEED_METERS_PER_SECOND;
            double ySpeedDelivered = ySpeedCommanded * Swerve.MAX_SPEED_METERS_PER_SECOND;
            double rotDelivered = currentRotation * Swerve.MAX_ANGULAR_SPEED;

            var swerveModuleStates = Swerve.DRIVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()))
                    : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
            SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, Swerve.MAX_SPEED_METERS_PER_SECOND);
            frontLeftModule.setDesiredState(swerveModuleStates[0]);
            frontRightModule.setDesiredState(swerveModuleStates[1]);
            backLeftModule.setDesiredState(swerveModuleStates[2]);
            backRightModule.setDesiredState(swerveModuleStates[3]);
        }
    
    }

    /**
   * Sets the wheels into an X formation to prevent movement.
   */
    public void setXState(boolean state) {
        xState = state;
        zState = false;
    }

    public void setZState(boolean state) {
        zState = state;
        xState = false;
    }



   /**
   * Sets the wheels into an X formation to prevent movement.
   */
    public void setToZero() {
        frontLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
        frontRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
        backLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
        backRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    }


   /**
   * Sets the swerve ModuleStates.
   * @param desiredStates The desired SwerveModule states.
   */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates, Swerve.MAX_SPEED_METERS_PER_SECOND);
        frontLeftModule.setDesiredState(desiredStates[0]);
        frontRightModule.setDesiredState(desiredStates[1]);
        backLeftModule.setDesiredState(desiredStates[2]);
        backRightModule.setDesiredState(desiredStates[3]);
    }

    /** Resets the turn encoders to a position of 0. */
    public void resetTurnEncoders() {
        frontLeftModule.resetTurnEncoder();
        frontRightModule.resetTurnEncoder();
        backLeftModule.resetTurnEncoder();
        backRightModule.resetTurnEncoder();
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        gyro.setYaw(0);
    }

   /**
   * Returns the heading of the robot.
   * @return the robot's heading in degrees, from -180 to 180
   */
    public double getHeading() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()).getDegrees();
    }

   /**
   * Returns the turn rate of the robot.
   * @return The turn rate of the robot, in degrees per second
   */
    public double getTurnRate() {
        return gyro.getYaw().getValueAsDouble() * (Swerve.GYRO_REVERSED ? -1.0 : 1.0);
    }

    /**
     * Drive for PathPlanner.
     * @param chassisSpeeds Chassis speed (-1 to 1).
     * @param limited Whether to enable rate limiting for smoother control.
     */
    public void pathPlannerDrive(ChassisSpeeds chassisSpeeds, boolean limited){
        double xSpeed = chassisSpeeds.vxMetersPerSecond / MAX_SPEED_METERS_PER_SECOND;
        double ySpeed = chassisSpeeds.vyMetersPerSecond / MAX_SPEED_METERS_PER_SECOND;
        double rot = chassisSpeeds.omegaRadiansPerSecond / MAX_ANGULAR_SPEED;

        double xSpeedCommanded;
        double ySpeedCommanded;
        if (limited) {
            // Convert XY to polar for rate limiting
            double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
            double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

            // Calculate the direction slew rate based on an estimate of the lateral acceleration
            double directionSlewRate;
            if (currentTranslationMag != 0.0) {
                directionSlewRate = Math.abs(Swerve.DIRECTION_SLEW_RATE / currentTranslationMag);
            } else {
                directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
            }
            
            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - prevTime;
            double angleDif = angleDifference(inputTranslationDir, currentTranslationDir);
            if (angleDif < 0.45*Math.PI){
                currentTranslationDir = stepTowardsCircular(currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
                currentTranslationMag = magLimiter.calculate(inputTranslationMag);
            } else if (angleDif > 0.85*Math.PI) {
                if (currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
                    // keep currentTranslationDir unchanged
                    currentTranslationMag = magLimiter.calculate(0.0);
                } else {
                    currentTranslationDir = wrapAngle(currentTranslationDir + Math.PI);
                    currentTranslationMag = magLimiter.calculate(inputTranslationMag);
                }
                } else {
                    currentTranslationDir = stepTowardsCircular(currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
                    currentTranslationMag = magLimiter.calculate(0.0);
                }
                prevTime = currentTime;
                xSpeedCommanded = currentTranslationMag * Math.cos(currentTranslationDir);
                ySpeedCommanded = currentTranslationMag * Math.sin(currentTranslationDir);
                currentRotation = rotLimiter.calculate(rot);


        } else {
            xSpeedCommanded = xSpeed;
            ySpeedCommanded = ySpeed;
            currentRotation = rot;
        }
        

        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeedCommanded * Swerve.MAX_SPEED_METERS_PER_SECOND;
        double ySpeedDelivered = ySpeedCommanded * Swerve.MAX_SPEED_METERS_PER_SECOND;
        double rotDelivered = currentRotation * Swerve.MAX_ANGULAR_SPEED;

        var swerveModuleStates = Swerve.DRIVE_KINEMATICS.toSwerveModuleStates(
            new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates, Swerve.MAX_SPEED_METERS_PER_SECOND);
        frontLeftModule.setDesiredState(swerveModuleStates[0]);
        frontRightModule.setDesiredState(swerveModuleStates[1]);
        backLeftModule.setDesiredState(swerveModuleStates[2]);
        backRightModule.setDesiredState(swerveModuleStates[3]);
    }
    
    

    /**
     * Gets ChassisSpeed object based on current module states.
     * @return ChassisSpeed object.
     */
    public ChassisSpeeds getChassisSpeeds(){
        ChassisSpeeds speeds = Constants.Swerve.DRIVE_KINEMATICS.toChassisSpeeds(
            frontLeftModule.getState(), 
            frontRightModule.getState(),
            backLeftModule.getState(),
            backRightModule.getState());
        return speeds;
    }

    public void slowState(boolean state){
        slow = state;
    }

    //public void driveX(double speed){
    public void driveX(double power){
        drive(power, 0, 0, false, true);
        // frontLeftModule.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(0)));
        // frontRightModule.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(0)));
        // backLeftModule.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(0)));
        // backRightModule.setDesiredState(new SwerveModuleState(speed, Rotation2d.fromDegrees(0)));
    }

    public void setXSpeed(double speed){
        
    }

   public double getFrontMeterValue(){
        return (frontLeftModule.getFrontPositionMeters() + frontRightModule.getFrontPositionMeters()) / 2;
    }
}