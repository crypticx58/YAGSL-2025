package frc.robot.Utils;

import edu.wpi.first.math.util.Units;

public enum ArmPreset {
    Zero(new ArmConfiguration(JointType.Shoulder.initialValue+5, Units.inchesToMeters(23.499999459334248+0.5), JointType.Wrist.initialValue-25)),
    L1(new ArmConfiguration(82.26927947998047, Units.inchesToMeters(24.12201851371705), 73.37083435058594)),
    L2(new ArmConfiguration(82.26927947998047, Units.inchesToMeters(24.12201851371705), 73.37083435058594)),
    L3(new ArmConfiguration(93.98094177246094, Units.inchesToMeters(23.499999459334248), 13.02807331085205)),
    L4(new ArmConfiguration(89.66891479492188 , Units.inchesToMeters(51.20078409750631), 27.428071975708008)),
    LowAlgae(new ArmConfiguration(45.696929931640625+5, Units.inchesToMeters(28.515009898839036+2+1), -17.82908058166504-2)),
    HighAlgae(new ArmConfiguration(61.50768280029297+2, Units.inchesToMeters(41.94723339531365+1.5+1.5+0.5), -39.429046630859375)),
    Processor(new ArmConfiguration(0.5, Units.inchesToMeters(23.499999459334248), 41.82804489135742+1.5)),
    StartingAlgae(new ArmConfiguration(33.186, Units.inchesToMeters(23.499999459334248), -40.114+10)),
    FloorAlgae(new ArmConfiguration(2.895608901977539+2.75, Units.inchesToMeters(25.890877866369532-0.5), -11.31478500366211)),
    SlingshotAlgae(new ArmConfiguration(90, Units.inchesToMeters(51), -20)),
    CoralStationFeed(new ArmConfiguration(20.83612632751465-1, Units.inchesToMeters(26.163008269362564), 96.6852798461914));
    public final ArmConfiguration armConfiguration;
    private ArmPreset(ArmConfiguration armConfiguration){
        this.armConfiguration = armConfiguration;
    }
}
