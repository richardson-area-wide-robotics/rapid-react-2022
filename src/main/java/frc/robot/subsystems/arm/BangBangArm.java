package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BangBangArm extends SubsystemBase {

  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;

  private float REVERSE_LIMIT = (float) -26.5;
  private float FORWARD_LIMIT = (float) 0;
  private final double REVERSE_SPEED = -0.50;
  private final double FORWARD_SPEED = 0.37;
  private final double RAMPRATE = 0.1;
  private armPosition m_Position = armPosition.SCORING_ARM_POSITION;

  enum armPosition {
    INTAKE_ARM_POSITION,
    SCORING_ARM_POSITION
  }

  public BangBangArm(int rightMotorCANID, int leftMotorCANID) {
    this.rightMotor = new CANSparkMax(rightMotorCANID, MotorType.kBrushless);
    // this.rightMotor.setInverted(invertRightMotor);
    this.rightMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    this.rightMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, FORWARD_LIMIT);
    this.rightMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    this.rightMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, REVERSE_LIMIT);
    this.rightMotor.setOpenLoopRampRate(RAMPRATE);
    this.rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    this.leftMotor = new CANSparkMax(leftMotorCANID, MotorType.kBrushless);
    this.leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    // this.leftMotor.setInverted(invertLeftMotor);
    this.leftMotor.follow(this.rightMotor, true);
  }

  public void runToScore() {
    this.rightMotor.set(FORWARD_SPEED);
    this.m_Position = armPosition.SCORING_ARM_POSITION;
  }

  public void runToIntake() {
    this.rightMotor.set(REVERSE_SPEED);
    this.m_Position = armPosition.INTAKE_ARM_POSITION;
  }

  public double getPosition() {
    return this.rightMotor.getEncoder().getPosition();
  }

  public double getMeasurement() {
    return this.getPosition();
  }

  public Boolean atForwardLimit() {
    return this.rightMotor.getFault(CANSparkMax.FaultID.kSoftLimitFwd);
  }

  public double getSpeed() {
    return this.rightMotor.get();
  }

  public Boolean atReverseLimit() {
    return this.rightMotor.getFault(CANSparkMax.FaultID.kSoftLimitRev);
  }

  public void moveArmToPosition(armPosition position) {
    if (position == armPosition.INTAKE_ARM_POSITION) {
      this.runToScore();
    } else if (position == armPosition.SCORING_ARM_POSITION) {
      this.runToIntake();
    }
  }

  public void toggleArmPosition() {
    if (this.m_Position == armPosition.INTAKE_ARM_POSITION) {
      this.runToScore();
    } else {
      this.runToIntake();
    }
  }

  public boolean resetBangArmDownPosition() {
    this.rightMotor
        .getEncoder()
        .setPosition(
            REVERSE_LIMIT); // put the arm fully down. and set the encoder  to its full down
    // position
    return true;
  }

  public boolean resetBangArmUpPosition() {
    this.rightMotor
        .getEncoder()
        .setPosition(
            FORWARD_LIMIT); // put the arm fully down. and set the encoder  to its full down
    // position
    return true;
  }

  /*public void armPositioningManipulation() {
    if (resetBangArmPosition() == false){
      if (this.m_Position == armPosition.INTAKE_ARM_POSITION) {
        this.runToScore();
      } else {
        this.runToIntake();
      }
    }
    //if (resetBangArmPosition() == true){
      //REVERSE_LIMIT = (float) -26.5;
      //FORWARD_LIMIT = (float) 0;
    //  if (this.m_Position == armPosition.INTAKE_ARM_POSITION){
     //   this.runToScore();
     // } else {
     //   this.runToIntake();
     // }
  }*/

  public void inverseToggleArmPosition() {
    if (resetBangArmDownPosition() == true) {}
  }
}
