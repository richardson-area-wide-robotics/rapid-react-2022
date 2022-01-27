package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import com.kauailabs.navx.frc.AHRS;

public class Gyroscope {
    //Classes
    private AHRS ahrs; 
    //variables
    private double startTime;
    private double driftPerSecond;
    
    public Gyroscope(){     
        this.ahrs = new AHRS(SPI.Port.kMXP);
        this.calibrate();
    }

    public double getGyroAngle(){
        double runTime = Timer.getFPGATimestamp() - startTime;
        double drift = runTime * driftPerSecond;
        return this.ahrs.getAngle() - drift;
    }

    public void getRotation2d(){
        this.ahrs.getRotation2d();
    }

    public void getRate(){
        this.ahrs.getRate();
    }

    public void resetGyro(){
        this.ahrs.reset();
        this.startTime = Timer.getFPGATimestamp();
    }

    public void calibrate() {
        this.startTime = Timer.getFPGATimestamp();   
        double startAngle = ahrs.getAngle();
        try{
            Thread.sleep(1000);
        } catch(Exception e) {
        }
        this.driftPerSecond = (ahrs.getAngle() - startAngle)/(Timer.getFPGATimestamp() - startTime);
    }
}