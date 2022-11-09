// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.Config;

public class DriveSubsystem extends SubsystemBase {

    // Instance for the Singleton class
    private static DriveSubsystem currentInstance;

    // SparkMax's
    private CANSparkMax leftLeader, rightLeader, leftFollower, rightFollower;

    // Gyro
    private PigeonIMU pigeon;

    // DifferentialDrive for teleop control
    private DifferentialDrive differentialDrive; 
    
    // Odometry to track the robot's location on the field
    private DifferentialDriveOdometry odometry;

    // NetworkTable Values
    private NetworkTableEntry xOdometry, yOdometry, rotOdometry;

    // Function to get the instance of the singleton class
    public static DriveSubsystem getInstance() {
        if (currentInstance == null) {
            currentInstance = new DriveSubsystem();
        }
        return currentInstance;
    }

    private DriveSubsystem() {
        /*
         * Construct SparkMaxs and pass settings 
         */
        leftLeader = new CANSparkMax(Config.LEFT_LEADER_CANID, MotorType.kBrushless);
        rightLeader = new CANSparkMax(Config.RIGHT_LEADER_CANID, MotorType.kBrushless);
        leftFollower = new CANSparkMax(Config.LEFT_FOLLOWER_CANID, MotorType.kBrushless);
        rightFollower = new CANSparkMax(Config.RIGHT_FOLLOWER_CANID, MotorType.kBrushless);

        // Restore the SparkMax's to a know group of settings
        leftLeader.restoreFactoryDefaults();
        rightLeader.restoreFactoryDefaults();
        leftFollower.restoreFactoryDefaults();
        rightFollower.restoreFactoryDefaults();

        // Make the followers copy the leaders
        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);

        // Invert the motor direction
        leftLeader.setInverted(Config.LEFT_LEADER_INVERT);
        rightLeader.setInverted(Config.RIGHT_LEADER_INVERT);
        leftFollower.setInverted(Config.LEFT_FOLLOWER_INVERT);
        rightFollower.setInverted(Config.RIGHT_FOLLOWER_INVERT);

        // Set brake or coast mode. When the motor is doing nothing, brake mode applies a brake
        leftLeader.setIdleMode(Config.DRIVESUBSYTEM_IDLEMODE);
        rightLeader.setIdleMode(Config.DRIVESUBSYTEM_IDLEMODE);
        leftFollower.setIdleMode(Config.DRIVESUBSYTEM_IDLEMODE);
        rightFollower.setIdleMode(Config.DRIVESUBSYTEM_IDLEMODE);

        // Position conversion factor to change the units
        leftLeader.getEncoder().setPositionConversionFactor(Config.ROTATIONS_TO_METERS);
        rightLeader.getEncoder().setPositionConversionFactor(Config.ROTATIONS_TO_METERS);
        leftFollower.getEncoder().setPositionConversionFactor(Config.ROTATIONS_TO_METERS);
        rightFollower.getEncoder().setPositionConversionFactor(Config.ROTATIONS_TO_METERS);

        // Velocity conversion factor to change the units
        leftLeader.getEncoder().setVelocityConversionFactor(Config.RPM_TO_METERS_PER_SECOND);
        rightLeader.getEncoder().setVelocityConversionFactor(Config.RPM_TO_METERS_PER_SECOND);
        leftFollower.getEncoder().setVelocityConversionFactor(Config.RPM_TO_METERS_PER_SECOND);
        rightFollower.getEncoder().setVelocityConversionFactor(Config.RPM_TO_METERS_PER_SECOND);

        // Prevents the motors from drawing more voltage than the specified amount
        leftLeader.enableVoltageCompensation(Config.DRIVESUBSYSTEM_VOLTAGECOMP);
        rightLeader.enableVoltageCompensation(Config.DRIVESUBSYSTEM_VOLTAGECOMP);
        leftFollower.enableVoltageCompensation(Config.DRIVESUBSYSTEM_VOLTAGECOMP);
        rightFollower.enableVoltageCompensation(Config.DRIVESUBSYSTEM_VOLTAGECOMP);

        // Prevent the motors from drawing more current than the specified amount
        leftLeader.setSmartCurrentLimit(Config.DRIVESUBSYTEM_CURRENTLIMIT);
        rightLeader.setSmartCurrentLimit(Config.DRIVESUBSYTEM_CURRENTLIMIT);
        leftFollower.setSmartCurrentLimit(Config.DRIVESUBSYTEM_CURRENTLIMIT);
        rightFollower.setSmartCurrentLimit(Config.DRIVESUBSYTEM_CURRENTLIMIT);
        
        // Set the P constant for the PID controller
        leftLeader.getPIDController().setP(Config.kRamsetePGain, 1);
        rightLeader.getPIDController().setP(Config.kRamsetePGain, 1);
        leftFollower.getPIDController().setP(Config.kRamsetePGain, 1);
        rightFollower.getPIDController().setP(Config.kRamsetePGain, 1);
        

        /*
         * Construct other objects
         */
        pigeon = new PigeonIMU(Config.PIGEON_ID);

        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getCurrentAngle())); 

        differentialDrive = new DifferentialDrive(leftLeader, rightLeader);

        var table = NetworkTableInstance.getDefault().getTable("DrivetrainOdometry");
        xOdometry = table.getEntry("xOdometry");
        yOdometry = table.getEntry("yOdometry");
        rotOdometry = table.getEntry("rotOdometry"); 
        
    }

    /**
     * Arcade drive method for a differential drive robot.
     *
     * @param forward The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
     * @param steering The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
     *     positive.
     */
    public void arcadeDrive(double forward, double steering) {
        differentialDrive.arcadeDrive(forward, steering);
    }

    /**
     *  Stop the motors from moving until told to move again.
     */
    public void stopMotors() {
        leftLeader.stopMotor();
        rightLeader.stopMotor();
        leftFollower.stopMotor();
        rightFollower.stopMotor();
    }


    /**
     * Get the fused heading from the pigeon
     * 
     * @return Heading of the robot in degrees
     */
    private double getCurrentAngle() {
        return pigeon.getFusedHeading();
    }
    
    /**
     * Get the encoder data in meters
     */
    private double getLeftPosition() {
        return leftLeader.getEncoder().getPosition();
    }

    /**
     * Get the encoder data in meters
     */
    private double getRightPosition() {
        return rightLeader.getEncoder().getPosition();
    }

    @Override
    public void periodic() {

        /** 
         * Give heading (from gyro) and encoder data in meters to odometry to calculate a new robot pose.
         */
        Pose2d newPose = odometry.update(Rotation2d.fromDegrees(getCurrentAngle()), getLeftPosition(), getRightPosition());

        xOdometry.setDouble(newPose.getX());
        yOdometry.setDouble(newPose.getY());
        rotOdometry.setDouble(newPose.getRotation().getDegrees());
    }

    /**
     * Returns the a Pose2d of the current robot location
     * 
     * Odometry calculates a new pose every robot cycle and stores
     * the value so this method is only reading the stored value.
     * This means it already does only 1 hardware read every cycle instead of 
     * many things calling hardware redundantly.
     * 
     * @param Pose2d the current pose of the robot
     */
    public Pose2d getPose() { 
        return odometry.getPoseMeters();
    }

    /**
     * This method will return the heading from odometry
     * 
     * Odometry keeps track of the gyro heading and in relation to
     * the value it was reset to using an offset so it's important to ask
     * the odometry for the rotation instead of directly from the gyro.
     * 
     * @param Rotation2d The heading. Rotation2d has a .getDegrees() & a .getRadians() method.
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * Set the odometry to a given pose.
     * 
     * Necessary to do at the beginning of a match.
     * 
     * Very important to set the encoders to 0 when resetting odometry.
     * 
     * @param newPose New pose to set odometry to.
     */
    public void resetPose(Pose2d newPose) {
        leftLeader.getEncoder().setPosition(0);
        rightLeader.getEncoder().setPosition(0);
        odometry.resetPosition(newPose, Rotation2d.fromDegrees(getCurrentAngle()));
    }

    /**
     * Set the left and right volts of the drivetrain while running a small P loop to make sure 
     * the correct velocity is achieved.
     * 
     * This method of using SimpleMotorFeedforward and frc-characterization is copied from
     * RamseteCommand {@link RamseteCommand}. However, instead of running a PIDController in RamseteCommand, 
     * the PID loop is run on the SparkMax using ControlType.kVelocity. Only a P gain is required
     * since it only has to compensate for any error in SimpleMotorFeedforward.
     * 
     * See writeup on frc-characterization for more info on that equation and how to characterize.
     * 
     * @param leftVelocity Meters per second for the left side.
     * @param rightVelocity Meters per second for the right side.
     */
    public void setRamsete(double leftVolts, double rightVolts, double leftVel, double rightVel) {
        leftLeader.getPIDController().setReference(leftVel, ControlType.kVelocity, 1, leftVolts);
        rightLeader.getPIDController().setReference(rightVel, ControlType.kVelocity, 1, rightVolts);

        // Make sure motor safety knows the motors are being used
        differentialDrive.feed();

    }

    /**
     * Get the velocity of the left side of the drivetrain
     * @return meters/second
     */
    public double getLeftVelocity() {
        return leftLeader.getEncoder().getVelocity();
    }

    /**
     * Get the velocity of the right side of the drivetrain
     * @return meters/second
     */
    public double getRightVelocity() {
        return rightLeader.getEncoder().getVelocity();
    }

}
