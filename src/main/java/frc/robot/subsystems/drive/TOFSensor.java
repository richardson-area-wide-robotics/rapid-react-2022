package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
*Full data sheet https://drive.google.com/file/d/1TZaQEPTUOwr0a_-ehMSx1A6ZTU-sqt30/view?usp=sharing
*from datasheet 
*PWM output:
*Another form of output data for the module is the PWM output, which has a square wave period of 20 Hz. The high level corresponds to the measured distance
*Formula: Distance (mm) = High time (ms) * 100 = High time (us)/10
*For example, if the measured high time is 10000us, then Distance=10000/10=1000mm
*/

public class TOFSensor  extends SubsystemBase {
    //using the GY-53 TOF Sensor in PWM Mode
    //Distance in MM	= getOutput() * .05 *1000 * 100
    //Distance in inches	= Distance in MM /25.4
    private DutyCycle port;
    double targetDistance; // stored in mm


    public TOFSensor  (DigitalSource digitalSource) {
        this.port = new DutyCycle(digitalSource);
        this.targetDistance = 0;
        this.setName("TOF Sensor port: " + digitalSource.getPortHandleForRouting());
    }
    public TOFSensor (int PWMPort){ // declaration with an int for the PWM Port, This is mostlikely bad because you want the digital sources to be handled all at once somewhere else
       this(new DigitalInput(PWMPort));   
    }

    public void setTargetDistance(double targetDistance) {
        this.targetDistance = targetDistance;
    }
    void setTargetDistanceInches(double distance) {
        this.targetDistance = distance * 25.4;
    }
    boolean isAtTarget() {
        return Math.abs(this.targetDistance - this.getDistance()) < 0.5;
    }
    boolean closerThanTarget() {
        return this.getDistance() < this.targetDistance;
    }
    double getDistance() {
        return this.port.getOutput() * .05 *1000 * 100;
    }
    double getDistanceInches() {
        return this.getDistance() / 25.4;
    }
}
    

