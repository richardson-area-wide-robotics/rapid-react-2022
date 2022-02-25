package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class BangBangLift {

  private CANSparkMax liftMotor;

  private final float REVERSE_LIMIT = 0;
  private final float FORWARD_LIMIT = 0;
  private final double REVERSE_SPEED = 0;
  private final double FORWARD_SPEED = 0;
  private final double RAMPRATE = 0;

  public BangBangLift(int liftMotorCANID, Boolean invertLiftMotor) {
    this.liftMotor = new CANSparkMax(liftMotorCANID, MotorType.kBrushless);
    this.liftMotor.setInverted(invertLiftMotor);
    this.liftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    this.liftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, FORWARD_LIMIT);
    this.liftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    this.liftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, REVERSE_LIMIT);
    this.liftMotor.setOpenLoopRampRate(RAMPRATE);
    this.liftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  public void runToMidBar() {
    this.liftMotor.set(FORWARD_SPEED);
  }

  public void runToDeploy() {
    this.liftMotor.set(REVERSE_SPEED);
  }

  public double getPosition() {
    return this.liftMotor.getEncoder().getPosition();
  }

  public Boolean atForwardLimit() {
    return this.liftMotor.getFault(CANSparkMax.FaultID.kSoftLimitFwd);
  }

  public Boolean atReverseLimit() {
    return this.liftMotor.getFault(CANSparkMax.FaultID.kSoftLimitRev);
  }
}
