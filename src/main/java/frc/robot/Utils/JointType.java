package frc.robot.utils;

import edu.wpi.first.math.util.Units;

public enum JointType {
    Shoulder(5, 1/150, 100, new Bound(-1,145)),
    Telescopic(6, 1/5, Units.inchesToMeters(24), new Bound(Units.inchesToMeters(23.5), Units.inchesToMeters(52))),
    Wrist(7, 1/45, 180, new Bound(-95,100));
    public final int ID;
    public final double gearRatio;
    public final double maxSpeed; // DegreesPerSec for rotational or MetersPerSec for linear
    public final Bound rangeOfMotion; // Degrees for rotational or Meters for linear
    public final double lowerBound;
    public final double upperBound;
    private JointType(int ID, double gearRatio, double maxSpeed, Bound rangeOfMotion){
        this.ID = ID;
        this.gearRatio = gearRatio;
        this.maxSpeed = maxSpeed;
        this.rangeOfMotion = rangeOfMotion;
        this.lowerBound = rangeOfMotion.LowerBound;
        this.upperBound = rangeOfMotion.UpperBound;
    }
}
