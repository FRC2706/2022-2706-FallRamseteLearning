// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import frc.robot.subsystems.DriveSubsystem;

/**
 * The Config class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 */
public final class Config {

    /** 
     * Drivetrain data 
     */
    // Drivetrain CanIDs of the motors
    public static int LEFT_LEADER_CANID = 35;
    public static int RIGHT_LEADER_CANID = 33;
    public static int LEFT_FOLLOWER_CANID = 37;
    public static int RIGHT_FOLLOWER_CANID = 39;

    // CanID of the pigeon IMU
    public static int PIGEON_ID = 27;
    
    // Drivetrain inversions
    public static boolean LEFT_LEADER_INVERT = false;
    public static boolean RIGHT_LEADER_INVERT = true;
    public static boolean LEFT_FOLLOWER_INVERT = false;
    public static boolean RIGHT_FOLLOWER_INVERT = true;

    // TrackWidth-> Distance between the left and right side of the drivetrain. Crucial in DifferentialDriveKinematics, see below.
    public static double trackWidth = 0.51762;

    // Diameter of wheel & ticksPerRevolution used to convert meters into encoder ticks
    public static double drivetrainWheelDiameter = 0.1524; // 6 inches = 0.1524 meters

    // Kinematics-> Converts a velocity & angular velocity to left velocity and right velocity. This value can affect how accurate
    //                  the robot does turns in autonamous.
    public static DifferentialDriveKinematics differentialDriveKinematics = new DifferentialDriveKinematics(trackWidth);

    // Conversion Ratios for the SparkMax units
    public static double DRIVETRAIN_GEAR_RATIO = 10.71;
    public static double ROTATIONS_TO_METERS = Math.PI * drivetrainWheelDiameter / DRIVETRAIN_GEAR_RATIO;
    public static double RPM_TO_METERS_PER_SECOND = ROTATIONS_TO_METERS * 0.1;
    
    // Drivetrain idle mode and voltage/current limits
    public static double DRIVESUBSYSTEM_VOLTAGECOMP = 9;
    public static int DRIVESUBSYTEM_CURRENTLIMIT = 20; //80;
    public static IdleMode DRIVESUBSYTEM_IDLEMODE = IdleMode.kBrake;

    /** 
     * System Identification data
     */
    // ksVolts -> adds +ksVolts or -ksVolts to overcome static friction in the direction of motion.
    // kvVoltSecondsPerMeter -> Adds the values number of volts for every meter per second of velocity desired.
    // kaVoltSecondsSquaredPerMeter -> Adds the values number of volts for every meter per second squared of acceleration desired.
    public static double ksVolts = 0.17003;
    public static double kvVoltSecondsPerMeter = 3.1455;
    public static double kaVoltSecondsSquaredPerMeter = 0.29864;
    public static SimpleMotorFeedforward drivetrainFeedforwards = new SimpleMotorFeedforward(
        ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter);

    // P Gain -> Motor voltage to apply for every m/s of error
    public static double kRamsetePGain = 0.00082467;

    
    /** 
     * Generating Trajectories Data
     */
    // MaxVelocity -> When it plans the trajectories, it will plan to go to the given max velocity.
    // MaxAcceleration -> When it plans the trajectories, it will ramp up/ramp down by this amount.
    //                  It will not plan to accelerate faster than this value.
    public static double kMaxSpeedMetersPerSecond = 2.0;
    public static double kMaxAccelerationMetersPerSecondSquared = 2.5;

    // voltageConstraint -> It will use this voltage constraint to make sure that the desired voltage given to a specific motor is achievable.
    //                          It will not allow any planned trajectory to tell a motor to go more than 10 volts.
    //                          This number is not 12 volts because it needs room to compensate.
    public static TrajectoryConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Config.ksVolts,
                            Config.kvVoltSecondsPerMeter, Config.kaVoltSecondsSquaredPerMeter), Config.differentialDriveKinematics, 9); 

    // TrajectoryConfig -> This object holds max velocity, max acceleration, kinematics to make sure those are obeyed and constraints.
    public static TrajectoryConfig trajectoryConfig = new TrajectoryConfig(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(differentialDriveKinematics).addConstraint(autoVoltageConstraint); 

    /** 
     * Ramsete Logging 
     */
    // Folder for Ramsete's logging data, /U means usb port closest to centre of Robo Rio
    public static String usbLogFolder = "/U/data_captures/";
    public static boolean RAMSETE_ENABLE_LOGGING = false;

    // Supplier to get the instance of the drivetrain (in case the name is changed down the line)
    public static Supplier<DriveSubsystem> getRamseteDriveSubsystemInstance = () -> DriveSubsystem.getInstance(); 

}
