// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignAlgae extends Command {
  /** Creates a new AutoAlignAlgae. */
  final PIDController xTranslationalPidController = new PIDController(0.25, 0, 0);
  final PIDController yTranslationalPidController = new PIDController(0.2, 0, 0);
  final PIDController rotationalPidController = new PIDController(4, 0.00, 0);
  

  SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  VisionSubsystem visionSubsystem = VisionSubsystem.getInstance();

  public AutoAlignAlgae() {
    //rotationalPidController.enableContinuousInput(-180, 180);
    xTranslationalPidController.setTolerance(2);
    yTranslationalPidController.setTolerance(3);
    //translationalPidController.setIZone(Units.inchesToMeters(8));
    rotationalPidController.setTolerance(0.5);
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LimelightHelpers.setPipelineIndex("limelight-front", VisionConstants.NeuralDetectorPipelineIndex);

    xTranslationalPidController.reset();
    yTranslationalPidController.reset();
    rotationalPidController.reset();
    //rotationalPidController.setSetpoint(Units.radiansToDegrees(visionSubsystem.getClosestReefAprilTagPose().rotateBy(new Rotation3d(0,0,Units.degreesToRadians(180))).getRotation().getZ()));
    xTranslationalPidController.setSetpoint(4);
    yTranslationalPidController.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //LimelightHelpers.setPipelineIndex("limelight-front", VisionConstants.NeuralDetectorPipelineIndex);
    LimelightResults results = LimelightHelpers.getLatestResults(VisionConstants.FrontLimelightName);
    //System.out.println("running");
    //System.out.println(results);
    if (true){//(results.targets_Fiducials.length != 0) {
      //LimelightTarget_Fiducial tag = results.targets_Fiducials[0];
      //System.out.println(Units.radiansToDegrees(tag.getTargetPose_CameraSpace().getRotation().getY()));
      //System.out.println("YE");
      rotationalPidController.setSetpoint(0);
      //rotationalPidController.setSetpoint(swerveSubsystem.swerveDrive.getYaw().minus(Rotation2d.fromRadians(tag.getTargetPose_CameraSpace().getRotation().getY())).getDegrees());
      swerveSubsystem.swerveDrive.drive(
        new ChassisSpeeds(
          //rotationalPidController.atSetpoint() ? xTranslationalPidController.calculate(LimelightHelpers.getTA("limelight-front")):0,
          0,
          0,
          //rotationalPidController.atSetpoint() ? yTranslationalPidController.calculate(tag.tx):0,
          //Units.degreesToRadians(rotationalPidController.calculate()))));
          Units.degreesToRadians(rotationalPidController.calculate(LimelightHelpers.getTX("limelight-front")))));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LimelightHelpers.setPipelineIndex("limelight-front", VisionConstants.AprilTagPipelineIndex);
    swerveSubsystem.swerveDrive.lockPose();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rotationalPidController.atSetpoint() && xTranslationalPidController.atSetpoint() && yTranslationalPidController.atSetpoint();
  }
}
