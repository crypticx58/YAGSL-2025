package frc.robot.utils;

import edu.wpi.first.math.util.Units;

public enum ArmPreset {
    L2(new ArmConfiguration(91.20332336425781, Units.inchesToMeters(23.499999459334248), 64.28636169433594)),
    L3(new ArmConfiguration(97.75121307373047, Units.inchesToMeters(23.499999459334248), 12.095724105834961 )),
    L4(new ArmConfiguration(84.92161560058594 , Units.inchesToMeters(51.98642963499535), 23.143321990966797)),
    CoralStationFeed(new ArmConfiguration(-2.4373626708984375 , Units.inchesToMeters(23.499999459334248), 130.00033569335938));
    public final ArmConfiguration armConfiguration;
    private ArmPreset(ArmConfiguration armConfiguration){
        this.armConfiguration = armConfiguration;
    }
}
