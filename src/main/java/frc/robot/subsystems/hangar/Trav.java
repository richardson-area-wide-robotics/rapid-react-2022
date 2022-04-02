package frc.robot.subsystems.hangar;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Trav extends SubsystemBase {

  private CANSparkMax elevatorMotor;
  private CANSparkMax armMotor;
  private SparkMaxPIDController elevatorPIDController;
  private final double CLIMBHEIGHT = 85.27; // The height arm is needs to be to attach on mid bar --Needs to be double checked 
  private final double RELEASEHEIGHT = 0.0; // where we need to be to passover to the static hooks 
  private final double ARMREALEASEHEIGHT = 0; // where we need to be to "shoot" out the arm for next climb - find number
  private final double HOOKHEIGHT = 0; // where we need to be to latch on to next rung - find number
  private final float SOFTLIMIT_REVERSE = 0;
  private final float SOFTLIMIT_FORWARD = 0; // Max height of the elevator --Number need to change based on trav climb

  private final float ARMLIMIT_REVERSE = 0; // Arm start point // where arm always needs to be to start climbing
  private final float ARMLIMIT_FORWARD = 0; // Max extension of the arm // arm lineup position --Number need to change based on trav climb
  private final double REVERSE_SPEED = -0.50;
  private final double FORWARD_SPEED = 0.37;
  private double RAMPRATE = 0.1; // The rate at which the Arm will move

  private final double DEADBAND = 0.5; // the amount of play allow in our PID controller
  public double kP = 0.1;
  public double kI = 0.0004;
  public double kD = 1;
  public double kIz = 0;
  public double kFF = 0;
  public double kMaxOutput = 1;
  public double kMinOutput = -1;

  public Trav(
      int elevatorMotor_canID, int armMotor_canID) {
    elevatorMotor = new CANSparkMax(elevatorMotor_canID, MotorType.kBrushless);
    elevatorMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    elevatorMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, SOFTLIMIT_REVERSE);
    elevatorMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, SOFTLIMIT_FORWARD);
    elevatorPIDController = elevatorMotor.getPIDController();
    elevatorPIDController.setP(kP);
    elevatorPIDController.setI(kI);
    elevatorPIDController.setD(kD);
    elevatorPIDController.setIZone(kIz);
    elevatorPIDController.setFF(kFF);
    elevatorPIDController.setOutputRange(kMinOutput, kMaxOutput);
    armMotor = new CANSparkMax(armMotor_canID, MotorType.kBrushless);
    armMotor.setOpenLoopRampRate(RAMPRATE);
    armMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    armMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ARMLIMIT_REVERSE);
    armMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ARMLIMIT_FORWARD);
  }

  /** */

  public double getPosition() {
    return this.elevatorMotor.getEncoder().getPosition();
  }

  public void setPosition(double position) {
    this.elevatorPIDController.setReference(position, ControlType.kPosition);
  }

  public void runToMidHeight() {
    this.setPosition(CLIMBHEIGHT);
  }

  public void runToReleaseHeight() {
    this.setPosition(RELEASEHEIGHT);
  }

  public void runToArmReleaseHeight() {
    this.setPosition(ARMREALEASEHEIGHT);
  }

  public void runToHookHeight() {
    this.setPosition(HOOKHEIGHT);
  }

  public void runToLineUp() {
    this.armMotor.set(FORWARD_SPEED);
  }

  public void runToClimb() {
    this.armMotor.set(REVERSE_SPEED);
  }

  public boolean isAtZero() {
    return this.elevatorMotor.getEncoder().getPosition() <= DEADBAND;
  }

  public boolean isAtMidHeight() {
    return Math.abs(this.getPosition() - CLIMBHEIGHT) < 5;
  }

  public boolean isAtReleaseHeight() {
    return Math.abs(this.getPosition() - RELEASEHEIGHT) < DEADBAND;
  }

  public void smartDashboard() {
    Shuffleboard.getTab("Operator Controls")
    .getLayout("Hangar", BuiltInLayouts.kList)
    .withSize(4, 4)
    .add(
        "Elavator up for climb start height",
            new InstantCommand(() -> this.runToMidHeight())); // hangar.runToMidHeight(), hangar
Shuffleboard.getTab("Operator Controls")
    .getLayout("Hangar", BuiltInLayouts.kList)
    .withSize(4, 4)
    .add(
        "Raise Robot Up", new InstantCommand(() -> this.runToReleaseHeight())); // hangar.runToReleaseHeight(), hangar
Shuffleboard.getTab("Operator Controls")
    .getLayout("Hangar", BuiltInLayouts.kList)
    .withSize(4, 4)
    .add(
        "Run to Arm Realease Height", new InstantCommand(() -> this.runToArmReleaseHeight())); // hangar.runToArmReleaseHeight(), hangar
Shuffleboard.getTab("Operator Controls")
    .getLayout("Hangar", BuiltInLayouts.kList)
    .withSize(4, 4)
    .add(
        "run to Hook Height", new InstantCommand(() -> this.runToHookHeight())); // hangar.runToHookHeight(), hangar
  Shuffleboard.getTab("Operator Controls")
        .getLayout("Hangar", BuiltInLayouts.kList)
        .withSize(4, 4)
        .add("LineUp arm on next Rung", new InstantCommand(() -> this.runToLineUp())); // hangar.runToLineUp(), hangar
  
  /* this last one may need to be a parrallel command, for moving arm to climb position and moving elevator down to release height */
  Shuffleboard.getTab("Operator Controls")
        .getLayout("Hangar", BuiltInLayouts.kList)
        .withSize(4, 4)
        .add("Move Arm to position for climbing ", new InstantCommand(() -> this.runToClimb())); // hangar.runToClimb(), hangar
  }


  /** @param elevatorPosition inches or cms from lowest position */
  public void setElevatorPosition(double elevatorPosition) {
    elevatorPIDController.setReference(elevatorPosition, CANSparkMax.ControlType.kPosition);
  }
}
