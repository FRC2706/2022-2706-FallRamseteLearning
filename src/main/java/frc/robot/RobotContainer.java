// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ramsete.RamseteCommandMerge;
import frc.robot.config.Config;
import frc.robot.subsystems.DriveSubsystem; 

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private SendableChooser<Integer> m_autonomousRoutines; 
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        // Create Main Tab in Shuffleboard
        ShuffleboardTab tab = Shuffleboard.getTab("Auto");
        m_autonomousRoutines = new SendableChooser<Integer>();

        // Add autonomous commands to page
        m_autonomousRoutines.setDefaultOption("Do Nothing", 0);
        m_autonomousRoutines.addOption("Routine1", 1);
        m_autonomousRoutines.addOption("Routine2", 2);
        m_autonomousRoutines.addOption("Routine3", 3);

        tab.add("Routines", m_autonomousRoutines).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0).withSize(2,1);

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        Joystick driverStick = new Joystick(0);

        DriveSubsystem.getInstance().setDefaultCommand(new ArcadeDrive(driverStick));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        /**
         * Create auto routines
         */
        // Example trajectory - Drive forward
        Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(List.of(
            new Pose2d(),
            new Pose2d(3, 0, Rotation2d.fromDegrees(0))),
            getConfig(0, 0, false));
        RamseteCommandMerge ramsete1 = new RamseteCommandMerge(trajectory1, "ExampleTrajectory-DriveForward");

        Command routine1 = new SequentialCommandGroup(
            resetOdometry(ramsete1),
            ramsete1
        );


        // Example trajectory - Drive forward and left
        Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(List.of(
            new Pose2d(),
            new Pose2d(3, -0.5, Rotation2d.fromDegrees(-30))),
            getConfig(0, 0, false));
        RamseteCommandMerge ramsete2 = new RamseteCommandMerge(trajectory2, "ExampleTrajectory-ForwardAndLeft");

        Command routine2 = new SequentialCommandGroup(
            resetOdometry(ramsete2),
            ramsete2
        );


        // Example trajectory - Drive forward and turn left
        Trajectory trajectory3 = TrajectoryGenerator.generateTrajectory(List.of(
            new Pose2d(),
            new Pose2d(3, 1, Rotation2d.fromDegrees(0))),
            getConfig(0, 0, false));
        RamseteCommandMerge ramsete3 = new RamseteCommandMerge(trajectory3, "ExampleTrajectory-ForwardAnd45degLeft");

        Command routine3 = new SequentialCommandGroup(
            resetOdometry(ramsete3),
            ramsete3
        );
        
        /**
         * Select a routine and return it
         */
        int pathSelected = (Integer) m_autonomousRoutines.getSelected();

        if (pathSelected == 0) {
            return new InstantCommand(DriveSubsystem.getInstance()::stopMotors);

        } else if (pathSelected == 1) {
            return routine1;
            
        } else if (pathSelected == 2) {
            return routine2;

        } else if (pathSelected == 3) {
            return routine3;
        }  
        return new InstantCommand(DriveSubsystem.getInstance()::stopMotors);      
    }
    
    /**
     * Get Config will return a TrajectoryConfig object needed to generate trajectories
     * 
     * If you want to add extra constraints its important to create a new 
     * TrajectoryConfig object for them.
     * 
     * For the same reason its important to set the startVelocity, endVelocity and isReversed
     * or else a value from a previous config may be used.
     * 
     * @param startVelocity Meters per second the robot starts at.
     * @param endVelocity Meters per second to ramp down to at the end of the trajectory.
     * @param isReversed True if the robot should drive in reverse. Poses must be 180 deg offset.
     * @return TrajectoryConfig object to pass to TrajectoryGenerator
     */
    private TrajectoryConfig getConfig(double startVelocity, double endVelocity, boolean isReversed) {
        return Config.trajectoryConfig
            .setStartVelocity(startVelocity)
            .setEndVelocity(endVelocity)
            .setReversed(isReversed);
    }
    
    /**
     * Create a command that will reset the odometry to the initial pose of a given trajectory.
     * 
     * @param trajectory Trajectory to get the inital pose from.
     * @return Command to reset odometry
     */
    private Command resetOdometry(RamseteCommandMerge ramseteCommand) {
        Pose2d initalPose = ramseteCommand.getInitialPose();
        return new InstantCommand(() -> DriveSubsystem.getInstance().resetPose(initalPose));
    }

    /**
     * Create a command that will reset the odometry to the given pose
     * 
     * @param pose Pose to reset odometry to
     * @return Command to reset odometry
     */
    private Command resetOdometry(Pose2d pose) {
        return new InstantCommand(() -> DriveSubsystem.getInstance().resetPose(pose));
    }
}
