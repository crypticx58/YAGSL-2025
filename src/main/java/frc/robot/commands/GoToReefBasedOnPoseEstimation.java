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
import frc.robot.field.FieldConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.ArmPreset;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToReefBasedOnPoseEstimation extends Command {
  /** Creates a new GoToPresetReefPositionBasedOnApriltag. */
  SwerveSubsystem  swerveSubsystem = SwerveSubsystem.getInstance();
  ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
  VisionSubsystem visionSubsystem = VisionSubsystem.getInstance();
  Pose3d ClosestAprilTagPose;
  //ReefHeight reefHeight;
  ArmPreset armPreset;
  boolean leftSide;
  Pose3d reefPose;
  Pose2d swervePoseSetpoint;
  final PIDController translationalPidController = new PIDController(3.7, 0, 0);
  final PIDController rotationalPidController = new PIDController(2.75, 0.00, 0);

  public GoToReefBasedOnPoseEstimation(boolean leftSide, ArmPreset armPreset) {
    this.leftSide = leftSide;
    //this.reefHeight = reefHeight;
    this.armPreset = armPreset;
    rotationalPidController.enableContinuousInput(-180, 180);
    translationalPidController.setTolerance(Units.inchesToMeters(3));
    //translationalPidController.setIZone(Units.inchesToMeters(8));
    rotationalPidController.setTolerance(2.5);
    addRequirements(swerveSubsystem, armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ClosestAprilTagPose = visionSubsystem.getClosestReefAprilTagPose();
    double LeftRightOffset = leftSide?-FieldConstants.Reef.LeftRightOffsetFromCenterMeters:FieldConstants.Reef.LeftRightOffsetFromCenterMeters;
    Pose3d LeftRightReefPose = ClosestAprilTagPose.plus(
      new Transform3d(
        new Translation3d(0, LeftRightOffset,0),
        new Rotation3d()
        )
      );
    swervePoseSetpoint = LeftRightReefPose.plus(
      new Transform3d(
        new Translation3d(ArmConstants.CoralHorizontalScoringDistance, 0,0),
        new Rotation3d(0,0,0)
      )
    ).toPose2d();
    // reefPose = new Pose3d(
    //   new Translation3d(LeftRightReefPose.getX(), LeftRightReefPose.getY(), reefHeight.height), 
    //   new Rotation3d(LeftRightReefPose.getRotation().getX(), Units.degreesToRadians(reefHeight.pitch), LeftRightReefPose.getRotation().getY())
    //   ).plus(VisionConstants.IntakeOffsetFromReefBranch);
    
    translationalPidController.reset();
    rotationalPidController.reset();

    rotationalPidController.setSetpoint(swervePoseSetpoint.getRotation().getDegrees());
    translationalPidController.setSetpoint(0);
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
    swerveSubsystem.swerveDrive.setChassisSpeeds(swerveSubsystem.chassisSpeedsForSwerveSetpointWithPID(swervePoseSetpoint, translationalPidController, rotationalPidController));
    armSubsystem.setArmConfigurationOptimally(armPreset.armConfiguration);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.swerveDrive.lockPose();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armSubsystem.isArmAtDesiredConfiguration(armPreset.armConfiguration, 2.5, Units.inchesToMeters(1), 2.5) && translationalPidController.atSetpoint() && rotationalPidController.atSetpoint();
  }
}
