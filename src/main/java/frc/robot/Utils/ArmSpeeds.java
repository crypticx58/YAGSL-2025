package frc.robot.utils;

// DPS = Degress Per Sec
public class ArmSpeeds {
    public final double ShoulderSpeedDPS;
    public final double TelescopicSpeedMPS;
    public final double WristSpeedDPS;
    public ArmSpeeds(double shoulderSpeedDPS, double telescopicSpeedMPS, double wristSpeedDPS){
        this.ShoulderSpeedDPS = shoulderSpeedDPS;
        this.TelescopicSpeedMPS = telescopicSpeedMPS;
        this.WristSpeedDPS = wristSpeedDPS;
    }
}
