package frc.robot.subsystems.drive;

public class DrivingDeltas {
    private double forwardPower = 0;
    private double steeringPower = 0;

    public DrivingDeltas(double forwardPower, double steeringPower) {
        this.forwardPower = forwardPower;
        this.steeringPower = steeringPower;
    }

    public double getForwardPower() {
        return this.forwardPower;
    }

    public void setForwardPower(double forwardPower) {
        this.forwardPower = forwardPower;
    }

    public double getSteeringPower() {
        return this.steeringPower;
    }

    public void setSteeringPower(double steeringPower) {
        this.steeringPower = steeringPower;
    }
}
