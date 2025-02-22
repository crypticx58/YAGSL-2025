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
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.ArmConfiguration;
import frc.robot.utils.ArmPreset;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToCoralStationGrooveBasedOnPoseEstimation extends Command {
  SwerveSubsystem  swerveSubsystem = SwerveSubsystem.getInstance();
  ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
  VisionSubsystem visionSubsystem = VisionSubsystem.getInstance();
  Pose3d ClosestGroovePose;
  Pose2d swervePoseSetpoint;
  ArmConfiguration armConfiguration = ArmPreset.CoralStationFeed.armConfiguration;
  final PIDController translationalPidController = new PIDController(3.7, 0, 0);
  final PIDController rotationalPidController = new PIDController(2.75, 0.00, 0);

  public GoToCoralStationGrooveBasedOnPoseEstimation() {
    rotationalPidController.enableContinuousInput(-180, 180);
    translationalPidController.setTolerance(Units.inchesToMeters(2.5));
    //translationalPidController.setIZone(Units.inchesToMeters(8));
    rotationalPidController.setTolerance(2.5);
    addRequirements(swerveSubsystem, armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ClosestGroovePose = visionSubsystem.getClosestCoralStationGroovePose();
    swervePoseSetpoint = ClosestGroovePose.plus(
      new Transform3d(
        new Translation3d(ArmConstants.OptimalCoralStationFeedDistance, 0,0),
        new Rotation3d(0,0,Math.PI)
      )
    ).toPose2d();
    
    translationalPidController.reset();
    rotationalPidController.reset();

    rotationalPidController.setSetpoint(swervePoseSetpoint.getRotation().getDegrees());
    translationalPidController.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveSubsystem.swerveDrive.setChassisSpeeds(swerveSubsystem.chassisSpeedsForSwerveSetpointWithPID(swervePoseSetpoint, translationalPidController, rotationalPidController));
    armSubsystem.setArmConfigurationOptimally(armConfiguration);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.swerveDrive.lockPose();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armSubsystem.isArmAtDesiredConfiguration(armConfiguration, 2.5, Units.inchesToMeters(1), 2.5) && translationalPidController.atSetpoint() && rotationalPidController.atSetpoint();
  }
}
