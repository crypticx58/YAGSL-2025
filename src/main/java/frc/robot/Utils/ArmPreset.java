package frc.robot.Utils;

import edu.wpi.first.math.util.Units;

public enum ArmPreset {
    Zero(new ArmConfiguration(JointType.Shoulder.initialValue+5, Units.inchesToMeters(23.499999459334248+0.5), JointType.Wrist.initialValue-1)),
    L1(new ArmConfiguration(82.26927947998047, Units.inchesToMeters(24.12201851371705), 73.37083435058594)),
    L2(new ArmConfiguration(82.26927947998047, Units.inchesToMeters(24.12201851371705), 73.37083435058594)),
    L3(new ArmConfiguration(93.98094177246094, Units.inchesToMeters(23.499999459334248), 13.02807331085205)),
    L4(new ArmConfiguration(89.66891479492188 , Units.inchesToMeters(51.20078409750631), 27.428071975708008)),
    CoralStationFeed(new ArmConfiguration(20.83612632751465-1, Units.inchesToMeters(26.163008269362564), 96.6852798461914));
    public final ArmConfiguration armConfiguration;
    private ArmPreset(ArmConfiguration armConfiguration){
        this.armConfiguration = armConfiguration;
    }
}
