package frc.robot.utils;

public class ArmConfiguration {
    public final double ShoulderPosition, TelescopicPosition, WristPosition;
    
    public ArmConfiguration(double ShoulderPosition, double TelescopicPosition, double WristPosition){
        this.ShoulderPosition = ShoulderPosition;
        this.TelescopicPosition = TelescopicPosition;
        this.WristPosition = WristPosition;
    }
}
