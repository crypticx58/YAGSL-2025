package frc.robot.Utils;

import edu.wpi.first.math.util.Units;

public enum JointType {
    Shoulder(5, 1/150, 100, new Bound(-5,145), 0.5),
    Telescopic(6, 1/5, Units.inchesToMeters(24), new Bound(Units.inchesToMeters(23), Units.inchesToMeters(52.5)), Units.inchesToMeters(23.5)),
    Wrist(7, 1/5, 90, new Bound(-180,180), 180-48);
    public final int ID;
    public final double gearRatio;
    public final double maxSpeed; // DegreesPerSec for rotational or MetersPerSec for linear
    public final Bound rangeOfMotion; // Degrees for rotational or Meters for linear
    public final double lowerBound;
    public final double upperBound;
    public final double initialValue;
    private JointType(int ID, double gearRatio, double maxSpeed, Bound rangeOfMotion, double initialValue){
        this.ID = ID;
        this.gearRatio = gearRatio;
        this.maxSpeed = maxSpeed;
        this.rangeOfMotion = rangeOfMotion;
        this.lowerBound = rangeOfMotion.LowerBound;
        this.upperBound = rangeOfMotion.UpperBound;
        this.initialValue = initialValue;
    }
}
