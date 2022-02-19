// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Gyroscope;
import frc.robot.commands.autonomousCommands.TrajectoryTutCommandGroup;
import frc.robot.operatorInputs.Controls;
import frc.robot.operatorInputs.OperatorInputs;
import frc.robot.commands.autonomousCommands.AutonPathCommand;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private TrajectoryTutCommandGroup trajectoryTutCommand;
  private Drive drive;
  private Gyroscope gyro;
  private RobotContainer m_robotContainer;
  private Controls driverControls;
  private OperatorInputs operatorInputs;
  private AutonPathCommand path1Command;
  private AutonPathCommand path2Command;
  private AutonPathCommand path3Command;
  private SequentialCommandGroup autonCommandGroup;

 // Constants
  private final int JOYSTICK_PORT_DRIVER = 1;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    this.driverControls = new Controls(new Joystick(JOYSTICK_PORT_DRIVER));
    this.gyro = new Gyroscope();
    this.drive = new Drive(gyro);
    this.operatorInputs = new OperatorInputs(driverControls, drive);

    Shuffleboard.getTab("Drive") 
      .add("gyro angle value", this.gyro.getGyroAngle())
      .withWidget(BuiltInWidgets.kGyro)
      .getEntry();

    this.path1Command = new AutonPathCommand(drive, "PathWeaver/output/path1.wpilib.json");
    this.path2Command = new AutonPathCommand(drive, "PathWeaver/output/path2.wpilib.json");
    this.path3Command = new AutonPathCommand(drive, "PathWeaver/output/path3.wpilib.json");
    this.autonCommandGroup = new SequentialCommandGroup(this.path1Command, this.path2Command,this.path3Command);
  }
    
  private void updateSmartDashboardValues() {   
    SmartDashboard.putNumber("right encoder value", this.drive.getRightEncoderDistance());
    SmartDashboard.putNumber("left encoder value", this.drive.getLeftEncoderDistance());
    SmartDashboard.putNumber("gyro angle value", this.gyro.getGyroAngle());
  }

  
  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    this.updateSmartDashboardValues(); 
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    this.drive.resetOdometry(this.path1Command.trajectory.getInitialPose());

    this.autonCommandGroup.schedule();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    this.updateSmartDashboardValues();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
