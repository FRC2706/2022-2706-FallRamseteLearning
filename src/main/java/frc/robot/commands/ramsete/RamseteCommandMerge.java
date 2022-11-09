// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ramsete;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.config.Config;
import frc.robot.subsystems.DriveSubsystem;

public class RamseteCommandMerge extends RamseteCommand {
    private final RamseteControllerLogging m_loggingRamseteController;
    private final Trajectory m_trajectory;

    /**
     * Construct a RamseteCommandMerge to use RamseteCommand to drive a given trajectory
     * 
     * @param trajectory Path to follow.
     * @param loggingDataFileName A logging data file will be created on a usb drive mounted on
     *                            the RoboRio. Give it a good name to know what path it recorded.
     */
    public RamseteCommandMerge(Trajectory trajectory, String loggingDataFileName) { 
        this(trajectory, Config.getRamseteDriveSubsystemInstance.get(), new RamseteControllerLogging(loggingDataFileName));
    }

    /**
     * Construct a RamseteCommandMerge to store the instance of RamseteControllerLogging
     */
    private RamseteCommandMerge(Trajectory trajectory, DriveSubsystem driveBase, RamseteControllerLogging loggingRamseteController) {
        super(
            trajectory,
            driveBase::getPose,
            loggingRamseteController,
            Config.differentialDriveKinematics,
            loggingRamseteController::calculateFeedforwards,
            driveBase
        );
        m_loggingRamseteController = requireNonNullParam(loggingRamseteController, "loggingRamseteController", "RamseteCommand"); 
        m_trajectory = requireNonNullParam(trajectory, "trajectory", "RamseteCommand");
    }

    @Override
    public boolean isFinished() {
        // If RamseteCommand wants to end, stop logging
        if (super.isFinished()) {
            m_loggingRamseteController.stopLogging();
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            Config.getRamseteDriveSubsystemInstance.get().stopMotors();
        }
        super.end(interrupted);
    }

    public Pose2d getInitialPose() {
        return m_trajectory.getInitialPose();
    }
}
