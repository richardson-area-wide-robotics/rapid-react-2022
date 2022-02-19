package frc.robot.commands.autonomousCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import java.io.IOException;
import java.nio.file.Path;

public class AutonPathCommand{

  private Drive drive;
  private DifferentialDriveVoltageConstraint autoVoltageConstraint;
  private TrajectoryConfig config;
  private Trajectory trajectory;
  private RamseteCommand ramseteCommand;

  // values to rememeber for robot velocity= 3.66 - Acceleration= 1.83 (these are starting values
  // that was changed over time)
  private final double kMaxSpeedMetersPerSecond = 0.90;
  private final double kMaxAccelerationMetersPerSecondSquared = 0.40;
  private final double MAX_VOLTAGE = 10;
  
  public AutonPathCommand(Drive drive, String trajectoryJSON) {
    // Create a voltage constraint to ensure we don't accelerate too fast
    this.drive = drive;
    this.autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            MAX_VOLTAGE);

    this.config =
        new TrajectoryConfig(
                this.kMaxSpeedMetersPerSecond, this.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      this.trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

  }

  public RamseteCommand getRamseteCommand() { 
    
    return this.ramseteCommand =
        new RamseteCommand(
            trajectory,
            this.drive::getPose,
            new RamseteController(), // (this.kRamseteB, this.kRamseteZeta),
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

    
  }
  public void resetOdometryToPathStart(){
    this.drive.resetOdometry(this.trajectory.getInitialPose());
  }
}
