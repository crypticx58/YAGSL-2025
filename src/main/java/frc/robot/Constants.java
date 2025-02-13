// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final double ARM_SPEEDS_DEADBAND = 0.3;
    public static final double SWERVE_DEADBAND = 0.3;
  }

  public static class ArmConstants {
    public static final Pose3d BasePose3d = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
    public static final double WristJointLengthMeters = Units.inchesToMeters(4);// TODO
    public static final Transform3d RobotToShoulderJointTransform = new Transform3d(
        new Translation3d(-Units.inchesToMeters(10.5), Units.inchesToMeters(14), 0),
        new Rotation3d(0, 0, 0)); // TODO

    public static final Transform3d IntakeToCoralTransform = new Transform3d(
        new Translation3d(0, 0, 0),
        new Rotation3d(0, 0, 0)); // TODO; 
    //We assume that the note is in the intake and we have the transform from the attachment point of the wrist and intake to the
    //imaginary center of the note. This is relative to the perspective of normal vecter of the attachment point
  }

  public static class VisionConstants {
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static final Transform3d BaseToSideCameraTransform = new Transform3d(
        new Translation3d(0, 0, 0),
        new Rotation3d(0, 0, 0));
    public static final Transform3d RobotToPhotonCameraTransform3d = new Transform3d(
        new Translation3d(0, 0, 0),
        new Rotation3d(0, 0, 0));
  }
}
