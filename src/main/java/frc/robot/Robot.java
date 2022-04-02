// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.networktables.NetworkTableInstance;
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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.autonomousCommands.AutonPathCommand;
import frc.robot.commands.drivingCommands.RobotAimingCommand;
import frc.robot.operatorInputs.Controls;
import frc.robot.operatorInputs.OperatorInputs;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.arm.BangBangArm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Gyroscope;
import frc.robot.subsystems.hangar.Hangar;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.LightsController;
import frc.robot.subsystems.sensors.TOFSensor;
import frc.robot.subsystems.vision.LimeLight;

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
  public OperatorInputs operatorInputs;
  private RobotAimingCommand aimRobot;
  // private Arm arm;
  private BangBangArm bangArm;
  private Intake intake;
  private Hangar hangar;
  private Lights lights;
  private LimeLight limeLight;
  private VideoSource limelightCamera;
  private TOFSensor tofSensor;
  private LightsController lightsController;
  private AutonPathCommand rightSideIntake_intake;
  private AutonPathCommand rightSideIntake_intakeSingleCargo;
  private AutonPathCommand rightSideIntake_intakeFirstTaxi;
  private AutonPathCommand rightSideIntake_scoreSingleCargo;
  private AutonPathCommand rightSideIntake_score;
  private AutonPathCommand backUpPath;

  private AutonPathCommand leftSide_intake;
  private AutonPathCommand leftSide_score;
  private AutonPathCommand leftSide_taxi;
  private AutonPathCommand leftSide_terminal;
  private AutonPathCommand leftSide_terminal2;
  private AutonPathCommand leftSide_backUp;

  private AutonPathCommand rightSideScore_backupAndAlign;
  private AutonPathCommand rightSideScore_intakeTwoCargo;
  private AutonPathCommand rightSideScore_scoreCargo;
  private AutonPathCommand rightSideScore_scoreFirstTaxi;
  private AutonPathCommand rightSideScore_turnIntake;
  private AutonPathCommand hangarPath;

  private SequentialCommandGroup rightSideIntake;
  private SequentialCommandGroup rightSideScore;
  private SequentialCommandGroup leftSide;
  private SequentialCommandGroup backUp;
  private SequentialCommandGroup fourBallAuton;

  private SequentialCommandGroup autoClimber;

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
    this.limelightCamera =
        CameraServer.addServer("http://10.17.45.47:5801/").getSource();
    this.gyro = new Gyroscope();
    this.drive = new Drive(gyro);
    this.intake = new Intake(9, 2, false);
    this.bangArm = new BangBangArm(8, 7);
    this.aimRobot = new RobotAimingCommand();
    this.limeLight = new LimeLight();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
    this.hangar = new Hangar(10, 1, 0, 1, 2, 3);
    //this.lights = new Lights(9, 60, 50);
    //this.lightsController = new LightsController(this.lights, this.tofSensor/*, this.limeLight*/);
    // this.tofSensor = new TOFSensor(0);
    // this.tofSensor = new TOFSensor(2);
    new InstantCommand(() -> this.hangar.enableCompressor());
    this.operatorInputs =
        new OperatorInputs(
            driverControls, operatorControls, drive, bangArm, intake, hangar, aimRobot);
    this.hangarPath = new AutonPathCommand(drive, "hangarPath.wpilib.json");
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
    this.leftSide_terminal2 =
        new AutonPathCommand(drive, "leftside/leftSide_terminal2.wpilib.json");
    this.leftSide_backUp = new AutonPathCommand(drive, "leftside/leftSide_backUp.wpilib.json");
    this.backUpPath = new AutonPathCommand(drive, "backUpPath.wpilib.json");

    this.rightSideScore_backupAndAlign =
        new AutonPathCommand(drive, "rightSideScore/rightSideScore_backupAndAlign.wpilib.json");
    this.rightSideScore_intakeTwoCargo =
        new AutonPathCommand(drive, "rightSideScore/rightSideScore_intakeTwoCargo.wpilib.json");
    this.rightSideScore_scoreCargo =
        new AutonPathCommand(drive, "rightSideScore/rightSideScore_scoreCargo.wpilib.json");
    this.rightSideScore_scoreFirstTaxi =
        new AutonPathCommand(drive, "rightSideScore/rightSideScore_scoreFirstTaxi.wpilib.json");
    this.rightSideScore_turnIntake =
        new AutonPathCommand(drive, "rightSideScore/rightSideScore_turnIntake.wpilib.json");

    this.rightSideScore =
        new SequentialCommandGroup(
            new InstantCommand(() -> this.rightSideScore_backupAndAlign.resetOdometryToPathStart()),
            new InstantCommand(() -> this.intake.outtake(), intake),
            new WaitCommand(0.1),
            this.rightSideScore_backupAndAlign.getRamseteCommand(),
            new InstantCommand(() -> this.bangArm.toggleArmPosition(), bangArm),
            new ParallelCommandGroup(
                new InstantCommand(() -> this.intake.gather(), intake),
                this.rightSideScore_turnIntake.getRamseteCommand()),
            new WaitCommand(0.1),
            new InstantCommand(() -> this.intake.idle(), intake),
            new ParallelCommandGroup(
                new InstantCommand(() -> this.intake.gather(), intake),
                this.rightSideScore_intakeTwoCargo.getRamseteCommand()),
            new WaitCommand(0.3),
            new InstantCommand(() -> this.intake.idle(), intake),
            new InstantCommand(() -> this.bangArm.toggleArmPosition(), bangArm),
            this.rightSideScore_scoreCargo.getRamseteCommand(),
            new InstantCommand(() -> this.intake.outtake(), intake),
            new WaitCommand(0.1),
            new InstantCommand(() -> this.intake.idle(), intake));

    this.backUp =
        new SequentialCommandGroup(
            new InstantCommand(() -> this.backUpPath.resetOdometryToPathStart()),
            new InstantCommand(() -> this.intake.outtake(), intake),
            new WaitCommand(0.2),
            new InstantCommand(() -> this.intake.idle(), intake),
            this.backUpPath.getRamseteCommand());

    this.leftSide =
        new SequentialCommandGroup(
            new InstantCommand(() -> this.leftSide_intake.resetOdometryToPathStart()),
            new InstantCommand(() -> this.bangArm.toggleArmPosition(), bangArm),
            new ParallelCommandGroup(
                new InstantCommand(() -> this.intake.gather(), intake),
                this.leftSide_intake.getRamseteCommand()),
            new InstantCommand(() -> this.intake.idle(), intake),
            new InstantCommand(() -> this.bangArm.toggleArmPosition(), bangArm),
            this.leftSide_score.getRamseteCommand(),
            new InstantCommand(() -> this.intake.outtake(), intake),
            new WaitCommand(0.2),
            new InstantCommand(() -> this.intake.idle(), intake),
            new InstantCommand(() -> this.leftSide_taxi.resetOdometryToPathStart()),
            this.leftSide_taxi.getRamseteCommand());

    this.fourBallAuton =
        new SequentialCommandGroup(
            new InstantCommand(() -> this.leftSide_intake.resetOdometryToPathStart()),
            new InstantCommand(() -> this.bangArm.toggleArmPosition(), bangArm),
            new ParallelCommandGroup(
                new InstantCommand(() -> this.intake.gather(), intake),
                this.leftSide_intake.getRamseteCommand()),
            new InstantCommand(() -> this.intake.idle(), intake),
            new InstantCommand(() -> this.bangArm.toggleArmPosition(), bangArm),
            this.leftSide_score.getRamseteCommand(),
            new InstantCommand(() -> this.intake.outtake(), intake),
            new WaitCommand(0.2),
            new InstantCommand(() -> this.intake.idle(), intake),
            new InstantCommand(() -> this.leftSide_backUp.resetOdometryToPathStart()),
            this.leftSide_backUp.getRamseteCommand(),
            this.leftSide_terminal.getRamseteCommand(),
            new InstantCommand(() -> this.leftSide_terminal2.resetOdometryToPathStart()),
            this.leftSide_terminal2.getRamseteCommand());

    this.rightSideIntake =
        new SequentialCommandGroup(
            new InstantCommand(() -> this.rightSideIntake_intake.resetOdometryToPathStart()),
            new InstantCommand(() -> this.bangArm.toggleArmPosition(), bangArm),
            new ParallelCommandGroup(
                this.rightSideIntake_intake.getRamseteCommand(),
                new InstantCommand(() -> this.intake.gather(), intake)),
            new WaitCommand(0.1),
            new InstantCommand(() -> this.intake.idle(), intake),
            new InstantCommand(() -> this.bangArm.toggleArmPosition(), bangArm),
            new InstantCommand(() -> this.rightSideIntake_score.resetOdometryToPathStart()),
            this.rightSideIntake_score.getRamseteCommand(),
            new InstantCommand(() -> this.intake.outtake(), intake),
            new WaitCommand(0.2),
            new InstantCommand(() -> this.intake.idle(), intake),
            this.rightSideIntake_intakeSingleCargo.getRamseteCommand(),
            new InstantCommand(() -> this.bangArm.toggleArmPosition(), bangArm),
            new WaitCommand(0.2),
            new InstantCommand(() -> this.intake.gather(), intake),
            new WaitCommand(0.3),
            new InstantCommand(() -> this.intake.idle(), intake));
    /*new InstantCommand(() -> this.bangArm.toggleArmPosition(), bangArm),
    this.rightSideIntake_scoreSingleCargo.getRamseteCommand(),
      new InstantCommand(() -> this.intake.outtake(), intake),
      new WaitCommand(0.2),
     new InstantCommand(() -> this.intake.idle(), intake),
     new InstantCommand(() -> this.rightSideIntake_intakeFirstTaxi.resetOdometryToPathStart()),
     this.rightSideIntake_intakeFirstTaxi.getRamseteCommand());*/

    /*driverControls.getLeftJoystickBumper().whenPressed(new SequentialCommandGroup(
         new InstantCommand(() -> this.hangarPath.resetOdometryToPathStart()),
         //new ParallelCommandGroup(
            // new InstantCommand(() -> hangar.runToMidHeight()),
             //new InstantCommand(() -> hangar.releaseMidHooks(), hangar)),
         this.hangarPath.getRamseteCommand()));
         //new InstantCommand(() -> hangar.runToReleaseHeight(), hangar),
         //new InstantCommand(() -> hangar.releaseFlippyHooks(), hangar),
         //new InstantCommand(() -> hangar.engageMidHooks(), hangar));)
    this.autoClimber =
         new SequentialCommandGroup(
             new InstantCommand(() -> this.hangarPath.resetOdometryToPathStart()),
             new ParallelCommandGroup(
                 new InstantCommand(() -> hangar.runToMidHeight()),
                 new InstantCommand(() -> hangar.releaseMidHooks(), hangar)),
             this.hangarPath.getRamseteCommand());
             //new InstantCommand(() -> hangar.runToReleaseHeight(), hangar),
             //new InstantCommand(() -> hangar.releaseFlippyHooks(), hangar),
             //new InstantCommand(() -> hangar.engageMidHooks(), hangar));*/

    this.autonomousChooser = new SendableChooser<>();
    this.autonomousChooser.setDefaultOption("Right Side Score", this.rightSideScore);
    //this.autonomousChooser.addOption("Four Ball Auton", this.fourBallAuton);
    //this.autonomousChooser.addOption("Right Side Intake", this.rightSideIntake);
    this.autonomousChooser.addOption("Left Side Auton", this.leftSide);
    this.autonomousChooser.addOption("back up auton", this.backUp);

    Shuffleboard.getTab("Operator Controls")
        .getLayout("Arm", BuiltInLayouts.kList)
        .withSize(2, 2)
        .add("Arm move Command", new InstantCommand(() -> bangArm.toggleArmPosition(), bangArm));
    Shuffleboard.getTab("Operator Controls")
        .getLayout("Arm", BuiltInLayouts.kList)
        .withSize(2, 2)
        .add(
            "Arm Down Reset Command",
            new InstantCommand(() -> bangArm.resetBangArmDownPosition(), bangArm));
    Shuffleboard.getTab("Operator Controls")
        .getLayout("Arm", BuiltInLayouts.kList)
        .withSize(2, 2)
        .add(
            "Arm up Reset Command",
            new InstantCommand(() -> bangArm.resetBangArmUpPosition(), bangArm));
    Shuffleboard.getTab("Operator Controls")
        .getLayout("Intake", BuiltInLayouts.kList)
        .withSize(2, 2)
        .add("Intake", new InstantCommand(() -> intake.gather(), intake));
    Shuffleboard.getTab("Operator Controls")
        .getLayout("Intake", BuiltInLayouts.kList)
        .withSize(2, 2)
        .add("Outake", new InstantCommand(() -> intake.outtake(), intake));
    Shuffleboard.getTab("Operator Controls")
        .getLayout("Intake", BuiltInLayouts.kList)
        .withSize(2, 2)
        .add("Motors Idle", new InstantCommand(() -> intake.idle(), intake));

    // Put the chooser on the dashboard
    SmartDashboard.putData(this.autonomousChooser);
  }

  private void updateSmartDashboardValues() {
    SmartDashboard.putNumber("right encoder value", this.drive.getRightEncoderDistance());
    SmartDashboard.putNumber("left encoder value", this.drive.getLeftEncoderDistance());
    SmartDashboard.putNumber("Arm Position", this.bangArm.getPosition());
    SmartDashboard.putNumber("Analog Value", this.hangar.getAnalogVoltage());
    SmartDashboard.putNumber("Pressure of Anolog", this.hangar.getPressure());
    SmartDashboard.putNumber("gyro angle", this.gyro.getGyroAngle());
    SmartDashboard.putNumber("gyro rate", this.gyro.getRate());
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
    //this.updateSmartDashboardValues();
    this.limeLight.smartDashboard();
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
    //this.updateSmartDashboardValues();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // hangar.enableCompressor();
    this.limeLight.smartDashboard();
    new InstantCommand(() -> this.hangar.enableCompressor());
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    CameraServer.startAutomaticCapture();
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
