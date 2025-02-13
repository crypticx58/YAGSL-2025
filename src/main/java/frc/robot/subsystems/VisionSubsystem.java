// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.Constants.VisionConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
  public static VisionSubsystem INSTANCE;

  SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  private final PhotonCamera photonCamera;
  private final PhotonPoseEstimator photonPoseEstimator;
  private Matrix<N3, N1> curStdDevs;

  @SuppressWarnings("WeakerAccess")
  public static VisionSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new VisionSubsystem();
    }
    return INSTANCE;
  }

  private VisionSubsystem() {
    swerveSubsystem.swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
    photonCamera = new PhotonCamera("FrontCamera");
    photonPoseEstimator = new PhotonPoseEstimator(VisionConstants.aprilTagFieldLayout,
        PoseStrategy.CLOSEST_TO_REFERENCE_POSE, VisionConstants.RobotToPhotonCameraTransform3d);
  }

  public Optional<EstimatedRobotPose> getPhotonEstimatedGlobalPose() {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var change : photonCamera.getAllUnreadResults()) {
      visionEst = photonPoseEstimator.update(change);
      updateEstimationStdDevs(visionEst, change.getTargets());
    }
    return visionEst;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // First, tell Limelight your robot's current orientation
    Rotation2d robotYaw = swerveSubsystem.swerveDrive.getYaw();
    LimelightHelpers.SetRobotOrientation("", robotYaw.getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0);

    // Get the pose estimate
    LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("");

    // Add it to your pose estimator
    swerveSubsystem.swerveDrive.addVisionMeasurement(
        limelightMeasurement.pose,
        limelightMeasurement.timestampSeconds);
    
    var visionEst = getPhotonEstimatedGlobalPose();
    visionEst.ifPresent(
        est -> {
          // Change our trust in the measurement based on the tags we can see
          var estStdDevs = getPhotonEstimationStdDevs();

          swerveSubsystem.swerveDrive.addVisionMeasurement(
              est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
        });
  }

  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs = VisionConstants.kSingleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = VisionConstants.kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an
      // average-distance metric
      for (var tgt : targets) {
        var tagPose = photonPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty())
          continue;
        numTags++;
        avgDist += tagPose
            .get()
            .toPose2d()
            .getTranslation()
            .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        curStdDevs = VisionConstants.kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1)
          estStdDevs = VisionConstants.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        curStdDevs = estStdDevs;
      }
    }
  }

  public Matrix<N3, N1> getPhotonEstimationStdDevs() {
    return curStdDevs;
  }
}
