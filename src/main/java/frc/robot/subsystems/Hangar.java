package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hangar extends SubsystemBase {

  private CANSparkMax elevatorMotor;
  private SparkMaxPIDController elevatorPIDController;
  private DoubleSolenoid midPneumatics;
  private DoubleSolenoid flippyArms;
  private final double MIDHEIGHT = 85.27; // where scoring at the mid height would be
  private final double FLIPPYHEIGHT = 3.01; // where we need to be to score the flippy hooks
  private final double RELEASEHEIGHT = 1.25; // where we need to be to score the release hooks
  private final double DEADBAND = 0.5; // the amount of play allow in our PID controller
  private final float SOFTLIMIT_REVERSE = 0;
  private final float SOFTLIMIT_FORWARD = 90; // Max height of the elevator
  public double kP = 0.1;
  public double kI = 0.0004;
  public double kD = 1;
  public double kIz = 0;
  public double kFF = 0;
  public double kMaxOutput = 1;
  public double kMinOutput = -1;
  public Compressor compressor;

  public Hangar(
      int elevatorMotor_canID,
      int pneumaticsModule_CanID,
      int midSolenoidChannel_forward,
      int midSolenoidChannel_reverse,
      int flippyArmsSolenoidChannel_forward,
      int flippyArmsSolenoidChannel_reverse) {
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
    midPneumatics =
        new DoubleSolenoid(
            PneumaticsModuleType.REVPH, midSolenoidChannel_forward, midSolenoidChannel_reverse);
    flippyArms =
        new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            flippyArmsSolenoidChannel_forward,
            flippyArmsSolenoidChannel_reverse);
    this.flippyArms.set(DoubleSolenoid.Value.kOff);
    compressor = new Compressor(pneumaticsModule_CanID, PneumaticsModuleType.REVPH);
    compressor.enableAnalog(110, 120);
  }

  /** */
  public void enableCompressor() {
    compressor.enableAnalog(110, 120);
  }

  public double getAnalogVoltage() {
    return compressor.getAnalogVoltage();
  }

  public double getPressure() {
    return compressor.getPressure();
  }

  public double getAnalogPressure() {
    return (250.0*(getAnalogVoltage()/5.0))-25.0;
  }

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

  public void runToZeroPosition(){}

  public void engageMidHooks() {
    this.midPneumatics.set(DoubleSolenoid.Value.kForward);
  }

  public void releaseMidHooks() {
    this.midPneumatics.set(DoubleSolenoid.Value.kReverse);
  }

  public void releaseFlippyHooks() {
    this.flippyArms.set(DoubleSolenoid.Value.kForward);
  }

  public boolean isAtZero() {
    return this.elevatorMotor.getEncoder().getPosition() <= DEADBAND;
  }

  public boolean isAtMidHeight() {
    return Math.abs(this.getPosition() - MIDHEIGHT) < 5;
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
