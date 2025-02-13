package frc.robot.Utils;

// DPS = Degress Per Sec
public class ArmSpeeds {
    public final double ShoulderSpeedDPS;
    public final double TelescopicSpeedMPS;
    public final double WristSpeedDPS;
    public final double IntakeSpeedPercent;
    public ArmSpeeds(double shoulderSpeedDPS, double telescopicSpeedMPS, double wristSpeedDPS, double intakeSpeedPercent){
        this.ShoulderSpeedDPS = shoulderSpeedDPS;
        this.TelescopicSpeedMPS = telescopicSpeedMPS;
        this.WristSpeedDPS = wristSpeedDPS;
        this.IntakeSpeedPercent = intakeSpeedPercent;
    }
}
