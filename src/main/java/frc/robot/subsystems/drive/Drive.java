package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase{

  private Gearbox leftGearbox;
  private Gearbox rightGearbox;
  private Gyroscope gyroscope;
  private DifferentialDrive differentialDrive;

  //variables
  private double desiredAngle;

  //constants
  private final double P_VALUE = .0025;
  private final double RAMP_RATE = 1;
  private final double QUICK_TURN_THROTTLE_DEADZONE = 0.1;
  // left gear box CAN ids
  private final int LEFT_BACK_CAN_ID = 11; 
  private final int LEFT_FRONT_CAN_ID = 12;
  private final int LEFT_MIDDLE_CAN_ID = 13;
  // right gear box CAN ids
  private final int RIGHT_BACK_CAN_ID = 4;
  private final int RIGHT_FRONT_CAN_ID = 5;
  private final int RIGHT_MIDDLE_CAN_ID = 6;

  private MotorType DRIVE_MOTOR_TYPE = MotorType.kBrushless;

  private boolean gyroDisabled = false;

  public Drive(Gyroscope gyroscope) {
    this.leftGearbox = new Gearbox(new Encoder(0, 1), new CANSparkMax(LEFT_BACK_CAN_ID, DRIVE_MOTOR_TYPE),
        new CANSparkMax(LEFT_FRONT_CAN_ID, DRIVE_MOTOR_TYPE), new CANSparkMax(LEFT_MIDDLE_CAN_ID, DRIVE_MOTOR_TYPE));


    this.rightGearbox = new Gearbox(new Encoder(2, 3), new CANSparkMax(RIGHT_BACK_CAN_ID, DRIVE_MOTOR_TYPE),
        new CANSparkMax(RIGHT_FRONT_CAN_ID, DRIVE_MOTOR_TYPE), new CANSparkMax(RIGHT_MIDDLE_CAN_ID, DRIVE_MOTOR_TYPE));

    this.leftGearbox.setRampRate(RAMP_RATE);
    this.rightGearbox.setRampRate(RAMP_RATE);
    
    this.differentialDrive = new DifferentialDrive(this.leftGearbox.getMotorControllerGroup(), 
                                              this.rightGearbox.getMotorControllerGroup());

    this.gyroscope = gyroscope;

    leftGearbox.setInverted(true);
  }

  private void setLeftPower(double power) {
    this.leftGearbox.setPower(power);
  }

  private void setRightPower(double power) {
    this.rightGearbox.setPower(power);
  }

  public void arcadeDrive(double throttle, double turnModifier){
    differentialDrive.arcadeDrive(throttle, turnModifier);
  }

  public void curvatureDrive(double throttle, double curvature) {
    if(throttle != 0.0 && curvature == 0.0 && !gyroDisabled) { //driving straight and no turn
      if(this.desiredAngle == Integer.MAX_VALUE) { //means robot just started driving straight
        this.desiredAngle = gyroscope.getGyroAngle(); 
      }
      curvature = this.getAngularError(desiredAngle) * P_VALUE; 
      this.setLeftPower(-(throttle - curvature));
      this.setRightPower(throttle + curvature);
    }
    else { // when robot isn't driving straight
      this.desiredAngle = Integer.MAX_VALUE; //if turn is greater than 0 or if robot is still
      differentialDrive.curvatureDrive(-throttle, curvature, (Math.abs(throttle) < QUICK_TURN_THROTTLE_DEADZONE));
    }
  }

  public double getAngularError(double desiredAngle) {  
    return  (gyroscope.getGyroAngle() - desiredAngle);
  }

  public void setBrakeMode(){
    leftGearbox.setBrakeMode();
    rightGearbox.setBrakeMode();
  }

  public void setCoastMode() {
    leftGearbox.setCoastMode();
    rightGearbox.setCoastMode();
  }

  public double straightTurnPower(double pValue) {
    double error = leftGearbox.getEncoderDistance() - rightGearbox.getEncoderDistance();
    double turnPower = error * pValue;
    return turnPower;
  }

  public void gyroDisabled(boolean gyroDisabled) {
    this.gyroDisabled = gyroDisabled;
  }
}