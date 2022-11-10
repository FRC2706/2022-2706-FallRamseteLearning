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




    /** 
     * System Identification data
     */


    
    
    /** 
     * Generating Trajectories Data
     */




    /** 
     * Ramsete Logging 
     */
    // Folder for Ramsete's logging data, /U means usb port closest to centre of Robo Rio
    public static String usbLogFolder = "/U/data_captures/";
    public static boolean RAMSETE_ENABLE_LOGGING = false;

    // Supplier to get the instance of the drivetrain (in case the name is changed down the line)
    public static Supplier<DriveSubsystem> getRamseteDriveSubsystemInstance = () -> DriveSubsystem.getInstance(); 

}
