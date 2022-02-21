package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BangBangArm extends SubsystemBase {

  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;

  private final float REVERSE_LIMIT = 0;
  private final float FORWARD_LIMIT = 0;
  private final double REVERSE_SPEED = 0;
  private final double FORWARD_SPEED = 0;
  private final double RAMPRATE = 0;

  public BangBangArm(
      int rightMotorCANID, int leftMotorCANID, Boolean invertRightMotor, Boolean invertLeftMotor) {
    this.rightMotor = new CANSparkMax(rightMotorCANID, MotorType.kBrushless);
    this.rightMotor.setInverted(invertRightMotor);
    this.rightMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    this.rightMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, FORWARD_LIMIT);
    this.rightMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    this.rightMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, REVERSE_LIMIT);
    this.rightMotor.setOpenLoopRampRate(RAMPRATE);

    this.leftMotor = new CANSparkMax(leftMotorCANID, MotorType.kBrushless);
    this.leftMotor.setInverted(invertLeftMotor);
    this.leftMotor.follow(this.rightMotor);
  }

  public void runToScore() {
    this.rightMotor.set(FORWARD_SPEED);
  }

  public void runToIntake() {
    this.rightMotor.set(REVERSE_SPEED);
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

  public void moveArmToPosition(Arm.armPosition position) {
    if (position == Arm.armPosition.INTAKE_ARM_POSITION) {
      this.runToScore();
    } else if (position == Arm.armPosition.SCORING_ARM_POSITION) {
      this.runToIntake();
    }
  }

  public void toggleArmPosition() {
    if (this.getSpeed() == FORWARD_SPEED) {
      this.runToIntake();
    } else // if its already in intake position go to score position. if its never been in a
    // position also go to score position.
    {
      this.runToScore();
    }
  }
}
