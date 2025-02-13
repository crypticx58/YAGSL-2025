// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();

  private SendableChooser<Command> autoChooser;
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverXbox =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY(),
                                                                () -> driverXbox.getLeftX())
                                                            .withControllerRotationAxis(()->driverXbox.getRightX())
                                                            .deadband(OperatorConstants.SWERVE_DEADBAND)
                                                            .scaleTranslation(0.75).scaleRotation(0.75)
                                                            .allianceRelativeControl(true);
  public RobotContainer() {
    //Configure the trigger bindings
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Routine", autoChooser);
    configureBindings();
  }

  private void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = swerveSubsystem.driveFieldOriented(driveAngularVelocity);
    driverXbox.b().onTrue(Commands.runOnce(()->swerveSubsystem.zeroGyro(), swerveSubsystem));
    swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    if (Robot.isSimulation())
    {
      driverXbox.a().onTrue(Commands.runOnce(() -> swerveSubsystem.swerveDrive.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    }

    // Quick code to test arm mechanism
    // SparkMax sparkmax = new SparkMax(2, MotorType.kBrushless);
    // swerveSubsystem.setDefaultCommand(Commands.run(()->{
    //   if (Math.abs(driverXbox.getLeftY())>0.3){
    //     sparkmax.set(-driverXbox.getLeftY()/2);
    //   } else {
    //     sparkmax.set(0);
    //   }
    // }, swerveSubsystem));
    
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
