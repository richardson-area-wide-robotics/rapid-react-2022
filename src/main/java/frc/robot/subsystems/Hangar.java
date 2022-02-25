package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hangar extends SubsystemBase {

  private CANSparkMax elevatorMotor;
  private SparkMaxPIDController elevatorPIDController;
  private DoubleSolenoid midPneumatics;
  private DoubleSolenoid flippyArms;
  private final double MIDHEIGHT = 0; // where scoring at the mid height would be
  private final double FLIPPYHEIGHT = 0; // where we need to be to score the flippy hooks
  private final double RELEASEHEIGHT = 0; // where we need to be to score the release hooks
  private final double DEADBAND = 0.5; // the amount of play allow in our PID controller
  private final float SOFTLIMIT_REVERSE = 0;
  private final float SOFTLIMIT_FORWARD = 20; // Max height of the elevator
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  public Hangar(
      int elevatorMotor_canID,
      int midSolenoidChannel_forward,
      int midSolenoidChannel_reverse,
      int flappyArmsSolenoidChannel_forward,
      int flappyArmsSolenoidChannel_reverse) {
    elevatorMotor = new CANSparkMax(elevatorMotor_canID, MotorType.kBrushless);
    elevatorMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    elevatorMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, SOFTLIMIT_REVERSE);
    elevatorMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, SOFTLIMIT_FORWARD);
    elevatorPIDController = elevatorMotor.getPIDController();
    midPneumatics =
        new DoubleSolenoid(
            PneumaticsModuleType.REVPH, midSolenoidChannel_forward, midSolenoidChannel_reverse);
    flippyArms =
        new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            flappyArmsSolenoidChannel_forward,
            flappyArmsSolenoidChannel_reverse);
    this.flippyArms.set(DoubleSolenoid.Value.kOff);
  }

  /** */
  public double getPosition() {
    return this.elevatorMotor.getEncoder().getPosition();
  }

  public void setPosition(double position) {
    this.elevatorPIDController.setReference(position, ControlType.kPosition);
  }

  public void runToMidHeight() {
    this.setPosition(MIDHEIGHT);
  }

  public void runToFlippyHeight() {
    this.setPosition(FLIPPYHEIGHT);
  }

  public void runToReleaseHeight() {
    this.setPosition(RELEASEHEIGHT);
  }

  public void engageMidHooks() {
    this.midPneumatics.set(DoubleSolenoid.Value.kForward);
  }

  public void releaseMidHooks() {
    this.midPneumatics.set(DoubleSolenoid.Value.kReverse);
  }

  public void releaseFlippyHooks() {
    this.flippyArms.set(DoubleSolenoid.Value.kForward);
  }

  public boolean isAtMidHeight() {
    return Math.abs(this.getPosition() - MIDHEIGHT) < DEADBAND;
  }

  public boolean isAtReleaseHeight() {
    return Math.abs(this.getPosition() - RELEASEHEIGHT) < DEADBAND;
  }

  public boolean isAtFlippyHeight() {
    return Math.abs(this.getPosition() - FLIPPYHEIGHT) < DEADBAND;
  }

  /**
   * directions TBD opens or closes solenoids depending on direction
   *
   * @param midOnOrOff (which direction are we rotating the hooks)
   */
  public void rotateHooksMid(boolean midOnOrOff) {
    if (midOnOrOff == true) {
      midPneumatics.set(DoubleSolenoid.Value.kForward);
    } else {
      flippyArms.set(DoubleSolenoid.Value.kReverse);
    }
  }

  /**
   * directions TBD opens or closes solenoids depending on direction
   *
   * @param highOnOrOff (which direction are we rotating the hooks)
   */
  public void rotateHooksHigh(boolean highOnOrOff) {
    if (highOnOrOff == true) {
      midPneumatics.set(DoubleSolenoid.Value.kForward);
    } else {
      flippyArms.set(DoubleSolenoid.Value.kReverse);
    }
  }

  /**
   * sensors not figured out yet
   *
   * @return whether the hooks are secured on the mid bar
   */
  public boolean isGrabbingMidBar() {
    return false;
  }

  /**
   * sensors not figured out yet
   *
   * @return whether the hooks are secured on the high bar
   */
  public boolean isGrabbingHighBar() {
    return false;
  }

  /**
   * sensors not figured out yet
   *
   * @return whether the robot is fully on the ground
   */
  public boolean isRobotOnGround() {
    return false;
  }

  /** @param elevatorPosition inches or cms from lowest position */
  public void setElevatorPosition(double elevatorPosition) {
    elevatorPIDController.setReference(elevatorPosition, CANSparkMax.ControlType.kPosition);
  }
}
