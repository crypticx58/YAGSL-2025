// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmControllerCommand;
import frc.robot.commands.GoToReefBasedOnPoseEstimation;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.ArmPreset;
import frc.robot.utils.JointType;
import frc.robot.utils.InputsManager.ForwardKinematicsInputsManager;
import swervelib.SwerveInputStream;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
  private final VisionSubsystem visionSubsystem = VisionSubsystem.getInstance();

  private SendableChooser<Command> autoChooser;
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverXbox = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController armXbox = new CommandXboxController(OperatorConstants.ARM_CONTROLLER_PORT);
  private final ForwardKinematicsInputsManager forwardKinematicsInputsManager = new ForwardKinematicsInputsManager(()->armXbox.getLeftY(), ()->-armXbox.getRightY(), ()->driverXbox.getRightTriggerAxis()-driverXbox.getLeftTriggerAxis());
  private final ArmControllerCommand armControllerCommand = new ArmControllerCommand(forwardKinematicsInputsManager);
  
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
      () -> -driverXbox.getLeftY(),
      () -> -driverXbox.getLeftX())
      .withControllerRotationAxis(() -> -driverXbox.getRightX())
      .deadband(OperatorConstants.SWERVE_DEADBAND)
      .scaleTranslation(0.45).scaleRotation(0.45)
      .allianceRelativeControl(false);

  public RobotContainer() {
    // Configure the trigger bindings
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Routine", autoChooser);
    configureBindings();
  }

  private void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = swerveSubsystem.driveFieldOriented(driveAngularVelocity);
    driverXbox.b().onTrue(Commands.runOnce(() -> swerveSubsystem.zeroGyro(), swerveSubsystem));
    driverXbox.x().whileTrue(new GoToReefBasedOnPoseEstimation(true, ArmPreset.L3));
    driverXbox.a().whileTrue(new GoToReefBasedOnPoseEstimation(false, ArmPreset.L3));

    armXbox.a().whileTrue(Commands.run(() -> {armSubsystem.setArmConfigurationOptimally(ArmPreset.CoralStationFeed.armConfiguration);}, armSubsystem));
    armXbox.y().onTrue(Commands.runOnce(()-> {
      System.out.println("---------------------");
      System.out.println("Shoulder: "+armSubsystem.getJointPosition(JointType.Shoulder));
      System.out.println("Telescopic: "+Units.metersToInches(armSubsystem.getJointPosition(JointType.Telescopic)));
      System.out.println("Wrist: "+armSubsystem.getJointPosition(JointType.Wrist));
    }));

    swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    armSubsystem.setDefaultCommand(armControllerCommand);
    // if (Robot.isSimulation()) {
    //   driverXbox.a().onTrue(
    //       Commands.runOnce(() -> swerveSubsystem.swerveDrive.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    // }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
