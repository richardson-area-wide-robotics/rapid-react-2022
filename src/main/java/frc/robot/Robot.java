// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomousCommands.AutonPathCommand;
import frc.robot.commands.autonomousCommands.TrajectoryTutCommandGroup;
import frc.robot.operatorInputs.Controls;
import frc.robot.operatorInputs.OperatorInputs;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Gyroscope;

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

  private AutonPathCommand rightSideIntake_intakeAndScore;
  private AutonPathCommand rightSideIntake_intakeSingleCargo;
  private AutonPathCommand rightSideIntake_intakeFirstTaxi;

  private AutonPathCommand rightSideScore_backupAndAlign;
  private AutonPathCommand rightSideScore_intakeTwoCargo;
  private AutonPathCommand rightSideScore_scoreCargo;
  private AutonPathCommand rightSideScore_scoreFirstTaxi;

  private SequentialCommandGroup rightSideIntake;
  private SequentialCommandGroup rightSideScore;

  private SendableChooser<Command> autonomousChooser;

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

    this.rightSideIntake_intakeAndScore =
        new AutonPathCommand(drive, "rightSideIntake/rightSideIntake_intakeAndScore.wpilib.json");
    this.rightSideIntake_intakeSingleCargo =
        new AutonPathCommand(
            drive, "rightSideIntake/rightSideIntake_intakeSingleCargo.wpilib.json");
    this.rightSideIntake_intakeFirstTaxi =
        new AutonPathCommand(drive, "rightSideIntake/rightSideIntake_intakeFirstTaxi.wpilib.json");

    this.rightSideScore_backupAndAlign =
        new AutonPathCommand(drive, "rightSideScore/rightSideScore_backupAndAlign.wpilib.json");
    this.rightSideScore_intakeTwoCargo =
        new AutonPathCommand(drive, "rightSideScore/rightSideScore_intakeTwoCargo.wpilib.json");
    this.rightSideScore_scoreCargo =
        new AutonPathCommand(drive, "rightSideScore/rightSideScore_scoreCargo.wpilib.json");
    this.rightSideScore_scoreFirstTaxi =
        new AutonPathCommand(drive, "rightSideScore/rightSideScore_scoreFirstTaxi.wpilib.json");

    this.rightSideScore =
        new SequentialCommandGroup(
            new InstantCommand(() -> this.rightSideScore_backupAndAlign.resetOdometryToPathStart()),
            this.rightSideScore_backupAndAlign.getRamseteCommand(),
            this.rightSideScore_intakeTwoCargo.getRamseteCommand(),
            this.rightSideScore_scoreCargo.getRamseteCommand(),
            this.rightSideScore_scoreFirstTaxi.getRamseteCommand());

    this.rightSideIntake =
        new SequentialCommandGroup(
            new InstantCommand(
                () -> this.rightSideIntake_intakeAndScore.resetOdometryToPathStart()),
            this.rightSideIntake_intakeAndScore.getRamseteCommand(),
            this.rightSideIntake_intakeSingleCargo.getRamseteCommand(),
            this.rightSideIntake_intakeFirstTaxi.getRamseteCommand());

    this.autonomousChooser = new SendableChooser<>();
    this.autonomousChooser.setDefaultOption("Right Side Score", this.rightSideScore);
    this.autonomousChooser.addOption("Right Side Intake", this.rightSideIntake);

    // Put the chooser on the dashboard
    SmartDashboard.putData(this.autonomousChooser);
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
    this.autonomousChooser.getSelected().schedule();
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
