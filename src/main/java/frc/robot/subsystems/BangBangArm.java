package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class BangBangArm extends SubsystemBase {

    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;

   final private float REVERSE_LIMIT = 0;
  final private float FORWARD_LIMIT = 0 ;
  final private double REVERSE_SPEED = 0;
  final private double FORWARD_SPEED = 0;
  final private double RAMPRATE =0;

  public BangBangArm( int rightMotorCANID,int leftMotorCANID,Boolean invertRightMotor, Boolean invertLeftMotor )
  {
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

  public void runToScore()
  {
    this.rightMotor.set(FORWARD_SPEED);
  }

  public void runToIntake()
  {
    this.rightMotor.set(REVERSE_SPEED);
  }

  public double getPosition()
  {
    return this.rightMotor.getEncoder().getPosition();
  }

  public Boolean atForwardLimit()
  {
    return this.rightMotor.getFault(CANSparkMax.FaultID.kSoftLimitFwd);
  }
  
  public Boolean atReverseLimit()
  {
    return this.rightMotor.getFault(CANSparkMax.FaultID.kSoftLimitRev);
  }

}



