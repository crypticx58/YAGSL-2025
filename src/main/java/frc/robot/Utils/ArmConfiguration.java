package frc.robot.Utils;

public class ArmConfiguration {
    public final double ShoulderPosition, TelescopicPosition, WristPosition;
    
    public ArmConfiguration(double ShoulderPosition, double TelescopicPosition, double WristPosition){
        this.ShoulderPosition = ShoulderPosition;
        this.TelescopicPosition = TelescopicPosition;
        this.WristPosition = WristPosition;
    }
    public double getJointPosition(JointType jointType){
        if (jointType == JointType.Shoulder) return ShoulderPosition;
        if (jointType == JointType.Telescopic) return TelescopicPosition;
        if (jointType == JointType.Wrist) return WristPosition;
        return 0;
    }
}
