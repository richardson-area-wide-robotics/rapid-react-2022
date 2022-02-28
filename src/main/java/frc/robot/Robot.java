// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomousCommands.AutonPathCommand;
import frc.robot.operatorInputs.Controls;
import frc.robot.operatorInputs.OperatorInputs;
import frc.robot.subsystems.BangBangArm;
import frc.robot.subsystems.Hangar;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Gyroscope;
import java.util.Map;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Drive drive;
  private Gyroscope gyro;
  private RobotContainer m_robotContainer;
  private Controls driverControls;
  private Controls operatorControls;
  private OperatorInputs operatorInputs;
  // private Arm arm;
  private BangBangArm bangArm;
  private Intake intake;
  private Hangar hangar;
  private AutonPathCommand rightSideIntake_intake;
  private AutonPathCommand rightSideIntake_intakeSingleCargo;
  private AutonPathCommand rightSideIntake_intakeFirstTaxi;
  private AutonPathCommand rightSideIntake_scoreSingleCargo;
  private AutonPathCommand rightSideIntake_score;

  private AutonPathCommand leftSide_intake;
  private AutonPathCommand leftSide_score;
  private AutonPathCommand leftSide_taxi;
  private AutonPathCommand leftSide_terminal;

  private AutonPathCommand rightSideScore_backupAndAlign;
  private AutonPathCommand rightSideScore_intakeTwoCargo;
  private AutonPathCommand rightSideScore_scoreCargo;
  private AutonPathCommand rightSideScore_scoreFirstTaxi;

  private SequentialCommandGroup rightSideIntake;
  private SequentialCommandGroup rightSideScore;
  private SequentialCommandGroup leftSide;

  private SendableChooser<Command> autonomousChooser;

  // Constants
  private final int JOYSTICK_PORT_DRIVER = 1;
  private final int JOYSTICK_PORT_OPERATOR = 0;

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
    this.operatorControls = new Controls(new Joystick(JOYSTICK_PORT_OPERATOR));
    this.gyro = new Gyroscope();
    this.drive = new Drive(gyro);
    // this.arm = new Arm();
    this.intake = new Intake(9, false);
    this.bangArm = new BangBangArm(8, 7);
    this.hangar = new Hangar(10, 1, 0, 1, 2, 3);
    new InstantCommand(() -> this.hangar.enableCompressor());
    this.operatorInputs =
        new OperatorInputs(driverControls, operatorControls, drive, bangArm, intake, hangar);
    // this.hangar.enableCompressor();
    this.rightSideIntake_intake =
        new AutonPathCommand(drive, "rightSideIntake/rightSideIntake_intake.wpilib.json");
    this.rightSideIntake_score =
        new AutonPathCommand(drive, "rightSideIntake/rightSideIntake_score.wpilib.json");
    this.rightSideIntake_intakeSingleCargo =
        new AutonPathCommand(
            drive, "rightSideIntake/rightSideIntake_intakeSingleCargo.wpilib.json");
    this.rightSideIntake_scoreSingleCargo =
        new AutonPathCommand(drive, "rightSideIntake/rightSideIntake_scoreSingleCargo.wpilib.json");
    this.rightSideIntake_intakeFirstTaxi =
        new AutonPathCommand(drive, "rightSideIntake/rightSideIntake_intakeFirstTaxi.wpilib.json");

    this.leftSide_intake = new AutonPathCommand(drive, "leftside/leftSide_intake.wpilib.json");
    this.leftSide_score = new AutonPathCommand(drive, "leftside/leftSide_score.wpilib.json");
    this.leftSide_taxi = new AutonPathCommand(drive, "leftside/leftSide_taxi.wpilib.json");
    this.leftSide_terminal = new AutonPathCommand(drive, "leftside/leftSide_terminal.wpilib.json");

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
            // new InstantCommand(() -> this.intake.outtake(), intake),
            this.rightSideScore_backupAndAlign.getRamseteCommand());
    // new InstantCommand(() -> this.bangArm.toggleArmPosition(), bangArm),
    // new ParallelCommandGroup(new InstantCommand(() -> this.intake.gather(), intake),
    // this.rightSideScore_intakeTwoCargo.getRamseteCommand());
    // new InstantCommand(() -> this.bangArm.toggleArmPosition(), bangArm),
    // this.rightSideScore_scoreCargo.getRamseteCommand(),
    // new InstantCommand(() -> this.intake.outtake(), intake),
    // this.rightSideScore_scoreFirstTaxi.getRamseteCommand());

    this.leftSide =
        new SequentialCommandGroup(
            new InstantCommand(() -> this.leftSide_intake.resetOdometryToPathStart()),
            // new InstantCommand(() -> this.bangArm.toggleArmPosition(), bangArm),
            this.leftSide_intake.getRamseteCommand());
    // new InstantCommand(() -> this.intake.gather(), intake))
    // new InstantCommand(() -> this.bangArm.toggleArmP,osition(), bangArm),
    // this.leftSide_score.getRamseteCommand(),
    // new InstantCommand(() -> this.intake.outtake(), intake)
    // this.leftSide_taxi.getRamseteCommand());

    this.rightSideIntake =
        new SequentialCommandGroup(
            new InstantCommand(() -> this.rightSideIntake_intake.resetOdometryToPathStart()),
            // new InstantCommand(() -> this.bangArm.toggleArmPosition(), bangArm),
            this.rightSideIntake_intake.getRamseteCommand(),
            // new InstantCommand(() -> this.intake.gather(), intake),
            // new InstantCommand(() -> this.bangArm.toggleArmPosition(), bangArm),
            this.rightSideIntake_score.getRamseteCommand());
    // new InstantCommand(() -> this.intake.outtake(), intake),
    // new InstantCommand(() -> this.bangArm.toggleArmPosition(), bangArm),
    // this.rightSideIntake_intakeSingleCargo.getRamseteCommand(),
    // new InstantCommand(() -> this.intake.gather(), intake),
    // new InstantCommand(() -> this.bangArm.toggleArmPosition(), bangArm),
    // this.rightSideIntake_scoreSingleCargo.getRamseteCommand(),
    // new InstantCommand(() -> this.intake.outtake(), intake)
    // this.rightSideIntake_intakeFirstTaxi.getRamseteCommand());

    this.autonomousChooser = new SendableChooser<>();
    this.autonomousChooser.setDefaultOption("Right Side Score", this.rightSideScore);
    this.autonomousChooser.addOption("Right Side Intake", this.rightSideIntake);
    this.autonomousChooser.addOption("Left Side Auton", this.leftSide);

    InstantCommand armUpCommand = new InstantCommand(() -> this.bangArm.runToScore());
    InstantCommand armDownCommand = new InstantCommand(() -> this.bangArm.runToIntake());
    InstantCommand commandIntake = new InstantCommand(() -> this.bangArm.runToIntake());
    InstantCommand commandOutake = new InstantCommand(() -> this.bangArm.runToIntake());

    Shuffleboard.getTab("Operator Controls")
        .getLayout("Arm", BuiltInLayouts.kList)
        .withSize(2, 2)
        .withProperties(Map.of("Label position", "HIDDEN"))
        .add(armUpCommand);
    //Shuffleboard.getTab("Operator Controls")
        //.getLayout("Arm")
        //.add(armDownCommand);
   /* Shuffleboard.getTab("Operator Controls")
        .getLayout("Intake", BuiltInLayouts.kList)
        .add(commandIntake);
    Shuffleboard.getTab("OperatorControls")
        .getLayout("Intake")
        .add(commandOutake);
    Shuffleboard.getTab("Constants")
        .add("gyro angle value", this.gyro.getGyroAngle())
        .withWidget(BuiltInWidgets.kGyro)
         .getEntry();#*/

    // Put the chooser on the dashboard
    SmartDashboard.putData(this.autonomousChooser);
  }

  private void updateSmartDashboardValues() {
    SmartDashboard.putNumber("right encoder value", this.drive.getRightEncoderDistance());
    SmartDashboard.putNumber("left encoder value", this.drive.getLeftEncoderDistance());
    SmartDashboard.putNumber("Arm Position", this.bangArm.getPosition());
    SmartDashboard.putNumber("Analog Value", this.hangar.getAnalogVoltage());
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
    new InstantCommand(() -> this.hangar.enableCompressor());
    // hangar.enableCompressor();
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
    // hangar.enableCompressor();
    new InstantCommand(() -> this.hangar.enableCompressor());
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
