// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ramsete;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.config.Config;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class RamseteControllerLogging extends RamseteController {

    // This must match the double[] in this objects calculate() method
    private final String[] m_loggerHeadings = new String[] {
        "Approx. Match Time",
        "odometryX", "odometryY", "odometryRot",
        "plannedX", "plannedY", "plannedRot",
        "errorX", "errorY", "errorRot",
        "plannedVelocity",
        "plannedAcceleration",
        "desiredLeftVel", "desiredRightVel",
        "measuredLeftVel", "measuredRightVel"
    };

    // NetworkTable Values
    private NetworkTableEntry xError, yError, rotError, xRef, yRef, rotRef, xOdometry, yOdometry, rotOdometry;

    private String m_loggingDataFileName;
    private SimpleCsvLogger usbLogger = new SimpleCsvLogger();
    private boolean loggerInitialized = false;


    // Feedforward calculations
    private Timer m_timer = new Timer();
    private double prevTime = 0;
    private double prevLeftVelocity = 0;
    private double prevRightVelocity = 0;
    
    public RamseteControllerLogging(String loggingDataFileName) {
        super();
        m_loggingDataFileName = loggingDataFileName;

        var table = NetworkTableInstance.getDefault().getTable("RamseteAutoError"); 
        xError = table.getEntry("xError");
        yError = table.getEntry("yError");
        rotError = table.getEntry("rotError"); 
        
        xRef = table.getEntry("xRef");
        yRef = table.getEntry("yRef");
        rotRef = table.getEntry("rotRef"); 

        xOdometry = table.getEntry("xOdometry");
        yOdometry = table.getEntry("yOdometry");
        rotOdometry = table.getEntry("rotOdometry"); 

        m_timer = new Timer();
        m_timer.reset();
        m_timer.start();
    }

    public void startLogging() {
        if (Config.RAMSETE_ENABLE_LOGGING) {
            usbLogger.init(m_loggingDataFileName, m_loggerHeadings);
        }
        m_timer.reset();
        m_timer.start();
    }

    public void stopLogging() {
        if (Config.RAMSETE_ENABLE_LOGGING) {
            usbLogger.close();
        }
        loggerInitialized = false;
        m_timer.stop();
    }

    /**
     * Interjects logging into ramsete
     * 
     * Adds some code on top of the RamseteController calculate() method to log the values passed through it.
     */
    @Override
    public ChassisSpeeds calculate( 
            Pose2d currentPose,
            Pose2d poseRef,
            double linearVelocityRefMeters,
            double angularVelocityRefRadiansPerSecond) {

        // Do the usual controller calculation that would normally happen
        ChassisSpeeds controllerCalculation = super.calculate(currentPose, poseRef, linearVelocityRefMeters, angularVelocityRefRadiansPerSecond);
        
        // Get the desired left and right speeds to store
        DifferentialDriveWheelSpeeds leftRightSpeeds = Config.differentialDriveKinematics.toWheelSpeeds(controllerCalculation);
        
        // Initialize the logger if not done yet
        if (loggerInitialized == false) {
            startLogging();
            loggerInitialized = true;
        }

        // Error between odometry and desired pose
        Pose2d poseError = poseRef.relativeTo(currentPose);
        
        if (Config.RAMSETE_ENABLE_LOGGING) {
            // This series of entries must match the string of headings at the top of this file
            usbLogger.writeData(
                Timer.getMatchTime(),
                currentPose.getX(), currentPose.getY(), currentPose.getRotation().getDegrees(),
                poseRef.getX(), poseRef.getY(), poseRef.getRotation().getDegrees(),
                poseError.getX(), poseError.getY(), poseError.getRotation().getDegrees(),
                linearVelocityRefMeters,
                angularVelocityRefRadiansPerSecond,
                leftRightSpeeds.leftMetersPerSecond, leftRightSpeeds.rightMetersPerSecond,
                Config.getRamseteDriveSubsystemInstance.get().getLeftVelocity(), Config.getRamseteDriveSubsystemInstance.get().getRightVelocity()            
            );
        }

        // Update networktables
        xError.setNumber(poseError.getX());
        yError.setNumber(poseError.getY());
        rotError.setNumber(poseError.getRotation().getDegrees());

        xRef.setNumber(poseRef.getX());
        yRef.setNumber(poseRef.getY());
        rotRef.setNumber(poseRef.getRotation().getDegrees());

        xOdometry.setNumber(currentPose.getX());
        yOdometry.setNumber(currentPose.getY());
        rotOdometry.setNumber(currentPose.getRotation().getDegrees());

        // Let ramsete continue on as normal 
        return controllerCalculation;
    }
    
    public void calculateFeedforwards(double leftVelocity, double rightVelocity) {

        // Calculate time since last call (deltaTime)
        double currentTime = m_timer.get();
        double deltaTime = currentTime - prevTime;
        prevTime = currentTime;

        // Throw out a time longer than a few robot cycles (in between ramsete commands this builds up)
        if (deltaTime > 0.1)  {
            return;
        }

        // Calculate acceleration
        double leftAcceleration = (leftVelocity - prevLeftVelocity) / deltaTime;
        double rightAcceleration = (rightVelocity - prevRightVelocity) / deltaTime;

        // Calculate feed forward value
        double leftFeedforward = Config.drivetrainFeedforwards.calculate(leftVelocity, leftAcceleration);
        double rightFeedforward = Config.drivetrainFeedforwards.calculate(rightVelocity, rightAcceleration);

        Config.getRamseteDriveSubsystemInstance.get().setRamsete(leftFeedforward, rightFeedforward,
                                               leftVelocity, rightVelocity);

        prevLeftVelocity = leftVelocity;
        prevRightVelocity = rightVelocity;
    
    }
}
