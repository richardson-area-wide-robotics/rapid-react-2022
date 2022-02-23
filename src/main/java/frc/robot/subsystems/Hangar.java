package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hangar extends SubsystemBase {

  private CANSparkMax elevatorMotor;
  private SparkMaxPIDController elevatorPIDController;
  private DoubleSolenoid midPneumatics;
  private DoubleSolenoid flappyArms;

  public Hangar(int elevatorMotor_canID, int midSolenoidChannel_forward, int midSolenoidChannel_reverse, 
  int flappyArmsSolenoidChannel_forward, int flappyArmsSolenoidChannel_reverse) {
    elevatorMotor = new CANSparkMax(elevatorMotor_canID, MotorType.kBrushless);
    elevatorMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    elevatorPIDController = elevatorMotor.getPIDController();
    midPneumatics = new DoubleSolenoid(PneumaticsModuleType.REVPH, midSolenoidChannel_forward, midSolenoidChannel_reverse);
    flappyArms = new DoubleSolenoid(PneumaticsModuleType.REVPH, flappyArmsSolenoidChannel_forward, flappyArmsSolenoidChannel_reverse);
  }

  /**
   * sends elevatorPower to elevatorMotor positive power --> go up negative power --> go down
   *
   * @param elevatorPower between -1 and 1
   */
  public void setElevatorPower(double elevatorPower) {
    elevatorMotor.set(elevatorPower);
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
      flappyArms.set(DoubleSolenoid.Value.kReverse);
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
      flappyArms.set(DoubleSolenoid.Value.kReverse);
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
