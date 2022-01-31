package frc.robot.commands.autonomousCommands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;

public class TrajectoryTutCommand extends SequentialCommandGroup {
    
    private Drive drive;
    private DifferentialDriveVoltageConstraint autoVoltageConstraint; 
    private TrajectoryConfig config;
    private Trajectory exampleTrajectory;
    private RamseteCommand ramseteCommand;
    
    private final double kMaxSpeedMetersPerSecond = 5.0;
    private final double kMaxAccelerationMetersPerSecondSquared = 2.0;
    private final double kRamseteB = 0.0;
    private final double kRamseteZeta = 0.0;
    
    public TrajectoryTutCommand(Drive drive) {
         // Create a voltage constraint to ensure we don't accelerate too fast
        this.drive = drive;
        this.autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            10);

        this.config = new TrajectoryConfig(
            this.kMaxSpeedMetersPerSecond,
            this.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint); 

        this.exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                // Pass config
                config);
        
        this.ramseteCommand = new RamseteCommand(
                    exampleTrajectory,
                    this.drive::getPose,
                    new RamseteController(),//(this.kRamseteB, this.kRamseteZeta),
                    new SimpleMotorFeedforward(
                        DriveConstants.ksVolts,
                        DriveConstants.kvVoltSecondsPerMeter,
                        DriveConstants.kaVoltSecondsSquaredPerMeter),
                    DriveConstants.kDriveKinematics,
                    this.drive::getWheelSpeeds,
                    new PIDController(DriveConstants.kPDriveVel, 0, 0),
                    new PIDController(DriveConstants.kPDriveVel, 0, 0),
                    // RamseteCommand passes volts to the callback
                    this.drive::tankDriveVolts,
                    this.drive);
                    this.drive.resetOdometry(exampleTrajectory.getInitialPose());
                    addCommands(this.ramseteCommand);

    }

    // @Override
    // public void initialize() {
    //     // Reset odometry to the starting pose of the trajectory.
    //     this.drive.resetOdometry(exampleTrajectory.getInitialPose());
    // }

    // @Override
    // public void execute() {
    //     this.ramseteCommand.andThen(() -> this.drive.tankDriveVolts(0, 0));
    // }

    // @Override
    // public boolean isFinished() {
    //     return this.ramseteCommand.isFinished();
    // }

    // @Override
    // public void end(boolean interrupted) {

    // }
}
