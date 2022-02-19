package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

/** A robot arm subsystem that moves with a motion profile. */
public class Arm extends ProfiledPIDSubsystem {
  private final int LEFT_CAN_ID = 7;
  private final int RIGHT_CAN_ID = 8;
  private MotorType ARM_MOTOR_TYPE = MotorType.kBrushless;
  private final double K_ARM_SCORING_RADS = 0.0;
  private final double K_ARM_INTAKE_RADS = 0.0;
  private double kPositionConversionFactor = 1.0;
  private static double kMaxAccelerationRadPerSecSquared = 0.0;
  private static double kMaxVelocityRadPerSecond = 0.0;
  private double kAVoltSecondSquaredPerRad = 0.0;
  private double kVVoltSecondPerRad = 0.0;
  private double kGVolts = 0.0;
  private double kSVolts = 0.0;
  private static double kP = 0.0;
  // private DigitalInput lowerPostionHomeSensor; //this is a planned upgrade
  private enum armPosition {
    INTAKE_ARM_POSITION,
    SCORING_ARM_POSITION
  }

  private armPosition goalArmPosition;

  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;

  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          this.kSVolts, this.kGVolts,
          this.kVVoltSecondPerRad, this.kAVoltSecondSquaredPerRad);

  /** Create a new ArmSubsystem. */
  public Arm() {
    super(
        new ProfiledPIDController(
            kP,
            0,
            0,
            new TrapezoidProfile.Constraints(
                kMaxVelocityRadPerSecond, kMaxAccelerationRadPerSecSquared)),
        0);

    this.leftMotor = new CANSparkMax(LEFT_CAN_ID, ARM_MOTOR_TYPE);
    this.rightMotor = new CANSparkMax(RIGHT_CAN_ID, ARM_MOTOR_TYPE);
    this.leftMotor.follow(rightMotor, true);

    this.rightMotor.getEncoder().setPositionConversionFactor(this.kPositionConversionFactor);
    // move arm to scoring position
    moveArmToPosition(armPosition.SCORING_ARM_POSITION);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    this.rightMotor.setVoltage(output + feedforward);
  }

  @Override
  public double getMeasurement() {
    return this.rightMotor.getEncoder().getPosition();
  }

  public void moveArmToPosition(armPosition position) {
    if (position == armPosition.INTAKE_ARM_POSITION) {
      setGoal(this.K_ARM_INTAKE_RADS);
    }
    if (position == armPosition.SCORING_ARM_POSITION) {
      setGoal(this.K_ARM_SCORING_RADS);
    }
    goalArmPosition = position;
  }

  public void toggleArmPosition() {
    if (goalArmPosition == armPosition.INTAKE_ARM_POSITION) {
      moveArmToPosition(armPosition.SCORING_ARM_POSITION);
    }
    if (goalArmPosition == armPosition.SCORING_ARM_POSITION) {
      moveArmToPosition(armPosition.INTAKE_ARM_POSITION);
    }
  }
}
