// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

/** Add your docs here. */
public class ArmOrder {
    public final JointType first, second, third;
    public final double firstTolerance, secondTolerance, thirdTolerance;
    public ArmOrder(JointType first, JointType second, JointType third, double firstTolerance, double secondTolerance, double thirdTolerance){
        this.first = first;
        this.second = second;
        this.third = third;

        this.firstTolerance = firstTolerance;
        this.secondTolerance = secondTolerance;
        this.thirdTolerance = thirdTolerance;
    }
}
