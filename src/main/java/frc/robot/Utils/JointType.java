package frc.robot.utils;

import java.util.ArrayList;

import edu.wpi.first.math.util.Units;

public enum JointType {
    Shoulder(1, 150, 1, new Bound(0,200)),
    Telescopic(2, 5, 1, new Bound(Units.inchesToMeters(25), Units.inchesToMeters(46.5))),
    Wrist(3, 50, 1, new Bound(0,270)),
    Intake(4, 1, 1, new Bound(0,360));
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
    // public int ID(){
    //     return id;
    // }
    // public double gearRatio(){
    //     return gearRatio;
    // }
    // public double lowerBound(){
    //     return rangeOfMotion.LowerBound;
    // }
    // public double upperBound(){
    //     return rangeOfMotion.UpperBound;
    // }
    //  public double maxSpeed(){
    //      return maxSpeed;
    // }
}
