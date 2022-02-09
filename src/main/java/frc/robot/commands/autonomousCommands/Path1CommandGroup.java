package frc.robot.commands.autonomousCommands;

import java.io.IOException;
import java.nio.file.Path;

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

public class Path1CommandGroup extends SequentialCommandGroup {

private Drive drive; 
private DifferentialDriveVoltageConstraint autoVoltageConstraint; 
private TrajectoryConfig config;
private Trajectory exampleTrajectory;
private RamseteCommand ramseteCommand;

public final double kMaxSpeedMetersPerSecond = 0.90;
public final double kMaxAccelerationMetersPerSecondSquared = 0.40;
public final double MAX_VOLTAGE = 10;
String trajectoryJSON = "paths/Path1.wpilib.json";
Trajectory trajectory = new Trajectory();

    public Path1CommandGroup(Drive drive) {
      // Create a voltage constraint to ensure we don't accelerate too fast
          this.drive = drive;
          this.autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            MAX_VOLTAGE);

          this.config = new TrajectoryConfig(
            this.kMaxSpeedMetersPerSecond,
            this.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint); 

          try {
          Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
          trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
          } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
          }  

          this.ramseteCommand = new RamseteCommand(
          trajectory,
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
          this.drive.resetOdometry(trajectory.getInitialPose());
          addCommands(this.ramseteCommand);
      }
  }
