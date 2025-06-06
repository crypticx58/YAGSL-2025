// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Utils.ArmConfiguration;
import frc.robot.Utils.ArmPreset;
import frc.robot.field.FieldConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToReefBasedOnPoseEstimation extends Command {
  /** Creates a new GoToPresetReefPositionBasedOnApriltag. */
  SwerveSubsystem  swerveSubsystem = SwerveSubsystem.getInstance();
  ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
  VisionSubsystem visionSubsystem = VisionSubsystem.getInstance();
  Pose3d ClosestAprilTagPose;
  //ReefHeight reefHeight;
  ArmConfiguration armConfig;
  boolean againstWall;
  Pose3d reefPose;
  Pose2d swervePoseSetpoint;

  public GoToReefBasedOnPoseEstimation(boolean againstWall) {
    this.againstWall = againstWall;
    //this.reefHeight = reefHeight;
    //this.armPreset = armPreset;
   
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ClosestAprilTagPose = visionSubsystem.getClosestReefAprilTagPose();
    double LeftRightOffset = Units.inchesToMeters(0.25);//leftSide?-FieldConstants.Reef.LeftRightOffsetFromCenterMeters:FieldConstants.Reef.LeftRightOffsetFromCenterMeters;
    double distanceOffset = Units.inchesToMeters(45);//againstWall?ArmConstants.AgainstReefWallDistance:ArmConstants.OffsetReefWallDistance;
    Pose3d LeftRightReefPose = ClosestAprilTagPose.plus(
      new Transform3d(
        new Translation3d(0, LeftRightOffset,0),
        new Rotation3d()
        )
      );
    swervePoseSetpoint = LeftRightReefPose.plus(
      new Transform3d(
        new Translation3d(distanceOffset, 0,0),
        new Rotation3d(0,0,Math.PI)
      )
    ).toPose2d();
    //armConfig = new ArmConfiguration(Math.asin(Math.abs(ClosestAprilTagPose.getZ()-swerveSubsystem.swerveDrive.getPose().getY())+Units.inchesToMeters(16)), Units.inchesToMeters(23.5),0);
    // reefPose = new Pose3d(
    //   new Translation3d(LeftRightReefPose.getX(), LeftRightReefPose.getY(), reefHeight.height), 
    //   new Rotation3d(LeftRightReefPose.getRotation().getX(), Units.degreesToRadians(reefHeight.pitch), LeftRightReefPose.getRotation().getY())
    //   ).plus(VisionConstants.IntakeOffsetFromReefBranch);
    
    swerveSubsystem.setSwervePoseSetpoint(swervePoseSetpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // This is method is to do it with math
    // double HeightOfReefPoseRelativeToShoulderJoint = reefHeight.height-ArmConstants.ShoulderJointHeightOffGroundMeters;
    // double LengthOfTelescopicArm = Math.sqrt(Math.pow(HeightOfReefPoseRelativeToShoulderJoint,2)+Math.pow(ArmConstants.CoralHorizontalScoringDistance,2));
    // double AngleOfArmRelativeToHorizontal = Units.radiansToDegrees(Math.asin(HeightOfReefPoseRelativeToShoulderJoint/LengthOfTelescopicArm)); // Robot is facing
    // double TrueAngleOfArm = 180-AngleOfArmRelativeToHorizontal; // This is relative to the front
    // double TrueAngleOfWrist = 180-ArmConstants.OptimalCoralScoringWristAngleDegrees;
    // armConfiguration = new ArmConfiguration(TrueAngleOfArm, LengthOfTelescopicArm, TrueAngleOfWrist);

    
    //
    swerveSubsystem.swerveDrive.setChassisSpeeds(swerveSubsystem.chassisSpeedsForSwerveSetpointWithPID(swervePoseSetpoint));
    //armSubsystem.setArmConfiguration(armConfig);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.swerveDrive.lockPose();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;//swerveSubsystem.swerveSetpointReached();
  }
}
