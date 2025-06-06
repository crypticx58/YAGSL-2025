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
import frc.robot.Utils.BargeTarget;
import frc.robot.Utils.ReefTarget;
import frc.robot.field.FieldConstants;
import frc.robot.field.FieldConstants.Reef;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToReefFieldTargetBasedOnPoseEstimation extends Command {
  SwerveSubsystem  swerveSubsystem = SwerveSubsystem.getInstance();
  ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
  VisionSubsystem visionSubsystem = VisionSubsystem.getInstance();
  Pose3d ClosestGroovePose;
  Pose2d swervePoseSetpoint;
  //BargeTarget bargeTarget;
  boolean isOffset;
  boolean isLeftSide;
  ArmConfiguration armConfiguration = ArmPreset.CoralStationFeed.armConfiguration;
  

  public GoToReefFieldTargetBasedOnPoseEstimation(boolean isLeftSide, boolean isOffset) {
    this.isOffset = isOffset;
    this.isLeftSide = isLeftSide;
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveSubsystem.currentFieldTarget = FieldConstants.getCoralTargetFromReefTarget(swerveSubsystem.getCurrentFieldTarget(), isLeftSide);
    Pose2d targetPose = swerveSubsystem.getCurrentFieldTarget().getTargetPose().plus(FieldConstants.getFieldTargetOffset(swerveSubsystem.getCurrentFieldTarget(), isOffset)).toPose2d();
    // swerveSubsystem.currentFieldTarget = ReefTarget.Coral;
    
    swervePoseSetpoint = targetPose;
    swerveSubsystem.setSwervePoseSetpoint(swervePoseSetpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveSubsystem.swerveDrive.setChassisSpeeds(swerveSubsystem.chassisSpeedsForSwerveSetpointWithPID(swervePoseSetpoint));
    //armSubsystem.setArmConfigurationOptimally(armConfiguration); Seperate driving controls from arm controls
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.swerveDrive.lockPose();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return swerveSubsystem.swerveSetpointReached();
  }
}
