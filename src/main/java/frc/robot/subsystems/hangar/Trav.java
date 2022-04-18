package frc.robot.subsystems.hangar;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Trav extends SubsystemBase {

  private CANSparkMax elevatorMotor;
  private CANSparkMax otherElevatorMotor;
  private CANSparkMax armMotor;
  private SparkMaxPIDController elevatorPIDController;
  private SparkMaxPIDController armPIDController;
  private final double CLIMBHEIGHT =
      235; // The height arm is needs to be to attach on mid bar --Needs to be double checked
  private final double RELEASEHEIGHT = 0; // where we need to be to passover to the static hooks
  private final double ARMREALEASEHEIGHT =
      200; // where we need to be to "shoot" out the arm for next climb - find number
  private final double HOOKHEIGHT =
      318; // maybe 101.0; // where we need to be to latch on to next rung - find number
  private final float SOFTLIMIT_REVERSE = 0;
  private final float SOFTLIMIT_FORWARD =
      318; // Max height of the elevator --Number need to change based on trav climb

  // 18 for can Id
  private final float ARMNORMALPOSITION = (float) 0.5;
  private final float ARMOUTPOSITION = (float) -1.86 * 5;
  private final float ARMCLAMPPOSITION = (float) -7.60;
  private final float ARMLIMIT_REVERSE =
      (float) -100.0; // Arm start point // where arm always needs to be to start climbing
  private final float ARMLIMIT_FORWARD =
      (float)
          0.6; // Max extension of the arm // arm lineup position --Number need to change based on
  // trav climb
  // private final double REVERSE_SPEED = -0.50;
  // private final double FORWARD_SPEED = 0.37;
  private double RAMPRATE = 0.1; // The rate at which the Arm will move

  private final double DEADBAND = 0.5; // the amount of play allow in our PID controller
  public double armkP = 0.1;
  public double armkI = 0.0004;
  public double armkD = 1;
  public double armkIz = 0;
  public double armkFF = 0;
  public double armkMaxOutput = 1;
  public double armkMinOutput = -1;

  public double elevatorkP = 1;
  public double elevatorkI = 0;
  public double elevatorkD = 0.1;
  public double elevatorkIz = 0;
  public double elevatorkFF = 0;
  public double elevatorkMaxOutput = 1;
  public double elevatorkMinOutput = -1;

  public Trav(int elevatorMotor_canID, int OtherelevatorMotor_canID, int armMotor_canID, boolean invertElevatorMotor) {
    elevatorMotor = new CANSparkMax(elevatorMotor_canID, MotorType.kBrushless);
    this.elevatorMotor.setInverted(invertElevatorMotor);
    otherElevatorMotor = new CANSparkMax(OtherelevatorMotor_canID, MotorType.kBrushless);
    otherElevatorMotor.follow(elevatorMotor, true);
    elevatorMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    elevatorMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, SOFTLIMIT_REVERSE);
    elevatorMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, SOFTLIMIT_FORWARD);
    armMotor = new CANSparkMax(armMotor_canID, MotorType.kBrushless);
    armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ARMLIMIT_FORWARD);
    armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ARMLIMIT_REVERSE);
    elevatorPIDController = elevatorMotor.getPIDController();
    elevatorPIDController.setP(elevatorkP);
    elevatorPIDController.setI(elevatorkI);
    elevatorPIDController.setD(elevatorkD);
    elevatorPIDController.setIZone(elevatorkIz);
    elevatorPIDController.setFF(elevatorkFF);
    elevatorPIDController.setOutputRange(elevatorkMinOutput, elevatorkMaxOutput);
    armPIDController = armMotor.getPIDController();
    armPIDController.setP(armkP);
    armPIDController.setI(armkI);
    armPIDController.setD(armkD);
    // armPIDController.setIZone(armkIz);
    armPIDController.setFF(armkFF);
    armPIDController.setOutputRange(armkMinOutput, armkMaxOutput);
    armMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
    armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ARMLIMIT_REVERSE);
    armMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ARMLIMIT_FORWARD);
    armMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  /** */
  public double getPosition() {
    return this.elevatorMotor.getEncoder().getPosition();
  }

  public void setPosition(double position) {
    this.elevatorPIDController.setReference(position, ControlType.kPosition);
  }

  public void setArmPosition(double position) {
    this.armPIDController.setReference(position, ControlType.kPosition);
  }

  public void runToMidHeight() {
    this.setPosition(CLIMBHEIGHT);
    this.setArmPosition(ARMNORMALPOSITION);
  }

  public void runToReleaseHeight() {
    this.setPosition(RELEASEHEIGHT);
    // this.setArmPosition(ARMNORMALPOSITION);
  }

  public void runToArmReleaseHeight() {
    this.setPosition(ARMREALEASEHEIGHT);
  }

  public void runToHookHeight() {
    this.setPosition(HOOKHEIGHT);
  }

  public void runToArmOutLineUp() {
    this.setArmPosition(ARMOUTPOSITION);
  }

  public void runToArmStraightClimb() {
    this.setArmPosition(ARMNORMALPOSITION);
  }

  public void runToArmClamp() {
    this.setArmPosition(ARMCLAMPPOSITION);
  }

  public double getArmPosition() {
    return this.armMotor.getEncoder().getPosition();
  }

  public double getarmUpMovementValue() {
    return this.getArmPosition() + 0.50;
  }

  public double getArmDownMovementValue() {
    return this.getArmPosition() - 0.70;
  }

  public void manualMoveArmIn() {
    this.setArmPosition(this.getarmUpMovementValue());
  }

  public void manualMoveArmOut() {
    this.setArmPosition(this.getArmDownMovementValue());
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
    // Shuffleboard.getTab("Operator Controls")
    //     .getLayout("Hangar", BuiltInLayouts.kList)
    //     .withSize(4, 4)
    //     .add(
    //         "3.Run to Arm Realease Height", new InstantCommand(() ->
    // this.runToArmReleaseHeight())); // hangar.runToArmReleaseHeight(), hangar
    // Shuffleboard.getTab("Operator Controls")
    //     .getLayout("Hangar", BuiltInLayouts.kList)
    //     .withSize(4, 4)
    //     .add(
    //         "5.run to Hook Height", new InstantCommand(() -> this.runToHookHeight()));
    // Shuffleboard.getTab("Operator Controls")
    //     .getLayout("Hangar", BuiltInLayouts.kList)
    //     .withSize(4, 4)
    //     .add(
    //         "Arm shoot out",new SequentialCommandGroup (new InstantCommand(() ->
    // runToArmOutLineUp()), new WaitCommand(.5),  new InstantCommand(() -> runToHookHeight())/*,
    // new WaitCommand(1.0), new InstantCommand(() -> runToArmReleaseHeight())*/)); //
    // hangar.runToLineUp(), hangar // hangar.runToHookHeight(), hangar
    Shuffleboard.getTab("Operator Controls")
        .getLayout("Hangar", BuiltInLayouts.kList)
        .withSize(4, 4)
        .add(
            "1.Climb and passover",
            new SequentialCommandGroup(
                new InstantCommand(() -> runToReleaseHeight()),
                new WaitCommand(2.7),
                new InstantCommand(
                    () ->
                        runToArmReleaseHeight()) /*, new WaitCommand(1.0), new InstantCommand(() -> runToArmReleaseHeight())*/)); // hangar.runToLineUp(), hangar
    Shuffleboard.getTab("Operator Controls")
        .getLayout("Hangar", BuiltInLayouts.kList)
        .withSize(4, 4)
        .add(
            "1.Climb and passover for bar climb",
            new SequentialCommandGroup(
                new InstantCommand(
                    () ->
                        runToReleaseHeight()))); /*, new WaitCommand(1.0), new InstantCommand(() -> runToArmReleaseHeight())*/ // hangar.runToLineUp(), hangar
    // Shuffleboard.getTab("Operator Controls")
    //       .getLayout("Hangar", BuiltInLayouts.kList)
    //       .withSize(4, 4)
    //       .add("6.LineUp arm on next Rung", new InstantCommand(() -> this.runToArmClamp()));
    /* this last one may need to be a parrallel command, for moving arm to climb position and moving elevator down to release height */
    Shuffleboard.getTab("Operator Controls")
        .getLayout("Hangar", BuiltInLayouts.kList)
        .withSize(4, 4)
        .add(
            "7.Move Arm to position for climbing ",
            new ParallelCommandGroup(
                new InstantCommand(() -> this.runToArmStraightClimb()),
                new InstantCommand(
                    () -> this.runToReleaseHeight()))); // hangar.runToClimb(), hangar
    // Shuffleboard.getTab("Operator Controls")
    // .getLayout("Hangar", BuiltInLayouts.kList)
    // .withSize(4, 4)
    // .add("Move Arm Out", new InstantCommand(() -> this.manualMoveArmOut()));
    // Shuffleboard.getTab("Operator Controls")
    // .getLayout("Hangar", BuiltInLayouts.kList)
    // .withSize(4, 4)
    // .add("Move Arm In", new InstantCommand(() -> this.manualMoveArmIn()));
  }

  /** @param elevatorPosition inches or cms from lowest position */
  public void setElevatorPosition(double elevatorPosition) {
    elevatorPIDController.setReference(elevatorPosition, CANSparkMax.ControlType.kPosition);
  }
}
