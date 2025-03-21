// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Utils.ArmOrder;
import frc.robot.Utils.ArmPreset;
import frc.robot.Utils.JointType;
import frc.robot.Utils.ReefTarget;
import frc.robot.Utils.InputsManager.ForwardKinematicsInputsManager;
import frc.robot.commands.ArmControllerCommand;
import frc.robot.commands.AutoAlignAlgae;
import frc.robot.commands.GoToArmPreset;
import frc.robot.commands.GoToCoralStationGrooveBasedOnPoseEstimation;
import frc.robot.commands.GoToProcessorBasedOnPoseEstimation;
import frc.robot.commands.GoToReefBasedOnPoseEstimation;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OutakeCommand;
import frc.robot.commands.ZeroArm;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import swervelib.SwerveInputStream;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
  private final VisionSubsystem visionSubsystem = VisionSubsystem.getInstance();

  private SendableChooser<Command> autoChooser;
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverXbox = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController armXbox = new CommandXboxController(OperatorConstants.ARM_CONTROLLER_PORT);
  private final ForwardKinematicsInputsManager forwardKinematicsInputsManager = new ForwardKinematicsInputsManager(()->armXbox.getLeftY(), ()->-armXbox.getRightY(), ()->armXbox.getLeftX());
  private final ArmControllerCommand armControllerCommand = new ArmControllerCommand(forwardKinematicsInputsManager);
  
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
      () -> -driverXbox.getLeftY(),
      () -> -driverXbox.getLeftX())
      .withControllerRotationAxis(() -> -driverXbox.getRightX())
      .deadband(OperatorConstants.SWERVE_DEADBAND)
      .scaleTranslation(0.45).scaleRotation(0.45)
      .allianceRelativeControl(false);

  // final SequentialCommandGroup baseSequentialCommandGroup = new SequentialCommandGroup(
  //     // new GoToReefBasedOnPoseEstimation(false),
  //     // new GoToArmPreset(ArmPreset.LowAlgae),
  //     Commands.runOnce(()->intakeSubsystem.toggleIntake(0.175)),
  //     new GoToReefBasedOnPoseEstimation(true),
  //     Commands.waitSeconds(1.5),
  //     new ParallelCommandGroup(
  //       new GoToReefBasedOnPoseEstimation(false),
  //       new ZeroArm()
  //     ),
  //     new ParallelCommandGroup(
  //       swerveSubsystem.pathfindToProcessor(true),
  //       new GoToArmPreset(ArmPreset.Processor)
  //     ),
  //     swerveSubsystem.pathfindToProcessor(false),
  //     Commands.runOnce(()->intakeSubsystem.toggleOutake(-0.25)),
  //     Commands.waitSeconds(0.5),
  //     Commands.runOnce(()->intakeSubsystem.toggleOutake(-0.25)),
  //     new ZeroArm()
  //     );

  public RobotContainer() {
    // Configure the trigger bindings
    //System.out.println(DriverStation.getAlliance().get());
    NamedCommands.registerCommand("ZeroArm", new ZeroArm());
    NamedCommands.registerCommand("ArmPresetLowAlgae", new GoToArmPreset(ArmPreset.LowAlgae));
    NamedCommands.registerCommand("ArmPresetHighAlgae", new GoToArmPreset(ArmPreset.HighAlgae));
    NamedCommands.registerCommand("ArmPresetProcessor", new GoToArmPreset(ArmPreset.Processor));
    NamedCommands.registerCommand("WaitHalfSecond", Commands.waitSeconds(0.5));
    NamedCommands.registerCommand("WaitFullSecond", Commands.waitSeconds(1));
    NamedCommands.registerCommand("WaitFullHalfSecond", Commands.waitSeconds(1.5));
    NamedCommands.registerCommand("CycleFront", new SequentialCommandGroup(
      swerveSubsystem.pathfindToReefTarget(ReefTarget.Front, true),
      new GoToArmPreset(ArmPreset.LowAlgae),
      swerveSubsystem.getBaseAutonSequentialCommandGroup()));
    NamedCommands.registerCommand("CycleFrontLeft", new SequentialCommandGroup(
      swerveSubsystem.pathfindToReefTarget(ReefTarget.FrontLeft, true),
      new GoToArmPreset(ArmPreset.HighAlgae),
      swerveSubsystem.getBaseAutonSequentialCommandGroup()));
    NamedCommands.registerCommand("CycleFrontRight", new SequentialCommandGroup(
      swerveSubsystem.pathfindToReefTarget(ReefTarget.FrontRight, true),
      new GoToArmPreset(ArmPreset.HighAlgae),
      swerveSubsystem.getBaseAutonSequentialCommandGroup()));
    NamedCommands.registerCommand("CycleBack", new SequentialCommandGroup(
      swerveSubsystem.pathfindToReefTarget(ReefTarget.Back, true),
      new GoToArmPreset(ArmPreset.HighAlgae),
      swerveSubsystem.getBaseAutonSequentialCommandGroup()));
    NamedCommands.registerCommand("CycleBackRight", new SequentialCommandGroup(
      swerveSubsystem.pathfindToReefTarget(ReefTarget.BackRight, true),
      new GoToArmPreset(ArmPreset.LowAlgae),
      swerveSubsystem.getBaseAutonSequentialCommandGroup()));
    NamedCommands.registerCommand("CycleBackLeft", new SequentialCommandGroup(
      swerveSubsystem.pathfindToReefTarget(ReefTarget.BackLeft, true),
      new GoToArmPreset(ArmPreset.LowAlgae),
      swerveSubsystem.getBaseAutonSequentialCommandGroup()));

    NamedCommands.registerCommand("GoToClosestReefAgaistWall", new GoToReefBasedOnPoseEstimation(true));
    NamedCommands.registerCommand("GoToClosestReefOffset", new GoToReefBasedOnPoseEstimation(false));
    NamedCommands.registerCommand("GoToProcessorOffset", new GoToProcessorBasedOnPoseEstimation(true));
    NamedCommands.registerCommand("GoToProcessorNoOffset", new GoToProcessorBasedOnPoseEstimation(false));

    NamedCommands.registerCommand("GoToProcessorOffsetPathfind", swerveSubsystem.pathfindToProcessor(true));
    NamedCommands.registerCommand("GoToProcessorNoOffsetPathfind",  swerveSubsystem.pathfindToProcessor(false));

    NamedCommands.registerCommand("GoToReefTargetFrontOffset", swerveSubsystem.pathfindToReefTarget(ReefTarget.Front, true));
    NamedCommands.registerCommand("GoToReefTargetFrontNoOffset", swerveSubsystem.pathfindToReefTarget(ReefTarget.Front, false));
    NamedCommands.registerCommand("GoToReefTargetFrontRightOffset", swerveSubsystem.pathfindToReefTarget(ReefTarget.FrontRight, true));
    NamedCommands.registerCommand("GoToReefTargetFrontRightNoOffset", swerveSubsystem.pathfindToReefTarget(ReefTarget.FrontRight, false));
    NamedCommands.registerCommand("GoToReefTargetFrontLeftOffset", swerveSubsystem.pathfindToReefTarget(ReefTarget.FrontLeft, true));
    NamedCommands.registerCommand("GoToReefTargetFrontLeftNoOffset", swerveSubsystem.pathfindToReefTarget(ReefTarget.FrontLeft, false));

    NamedCommands.registerCommand("GoToReefTargetBackOffset", swerveSubsystem.pathfindToReefTarget(ReefTarget.Back, true));
    NamedCommands.registerCommand("GoToReefTargetBackNoOffset", swerveSubsystem.pathfindToReefTarget(ReefTarget.Back, false));
    NamedCommands.registerCommand("GoToReefTargetBackRightOffset", swerveSubsystem.pathfindToReefTarget(ReefTarget.BackRight, true));
    NamedCommands.registerCommand("GoToReefTargetBackRightNoOffset", swerveSubsystem.pathfindToReefTarget(ReefTarget.BackRight, false));
    NamedCommands.registerCommand("GoToReefTargetBackLeftOffset", swerveSubsystem.pathfindToReefTarget(ReefTarget.BackLeft, true));
    NamedCommands.registerCommand("GoToReefTargetBackLeftNoOffset", swerveSubsystem.pathfindToReefTarget(ReefTarget.BackLeft, false));

    NamedCommands.registerCommand("Intake", Commands.runOnce(()->intakeSubsystem.toggleIntake(0.175)));
    NamedCommands.registerCommand("Outake", Commands.runOnce(()->intakeSubsystem.toggleOutake(-0.25)));
    NamedCommands.registerCommand("TurnOffIntake", Commands.runOnce(()->intakeSubsystem.turnOffIntake()));
    

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Routine", autoChooser);
    configureBindings();
  }

  private void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = swerveSubsystem.driveFieldOriented(driveAngularVelocity);

    driverXbox.b().onTrue(Commands.runOnce(() -> swerveSubsystem.zeroFieldOrientedHeading(driveAngularVelocity), swerveSubsystem));
    
    driverXbox.rightBumper().whileTrue(new GoToReefBasedOnPoseEstimation(false));
    driverXbox.leftBumper().whileTrue(new GoToReefBasedOnPoseEstimation(true));
    // driverXbox.rightTrigger(.5).whileTrue(new GoToProcessorBasedOnPoseEstimation(true));
    // driverXbox.leftTrigger(.5).whileTrue(new GoToProcessorBasedOnPoseEstimation(false));
    driverXbox.rightTrigger(.5).whileTrue(swerveSubsystem.pathfindToProcessor(true));
    driverXbox.leftTrigger(.5).whileTrue(swerveSubsystem.pathfindToProcessor(false));
    //driverXbox.a().whileTrue(new GoToCoralStationGrooveBasedOnPoseEstimation());
    // driverXbox.y().whileTrue(swerveSubsystem.pathfindToPose(new Pose2d(new Translation2d(11.508, 7.088), Rotation2d.fromDegrees(180.000))));
    
    armXbox.b().whileTrue(new ZeroArm().repeatedly());
    armXbox.povDown().whileTrue(new GoToArmPreset(ArmPreset.LowAlgae).repeatedly());
    armXbox.povRight().whileTrue(new GoToArmPreset(ArmPreset.Processor).repeatedly());
    armXbox.povUp().whileTrue(new GoToArmPreset(ArmPreset.HighAlgae).repeatedly());
    armXbox.povLeft().whileTrue(new GoToArmPreset(ArmPreset.StartingAlgae).repeatedly());
    armXbox.a().whileTrue(new GoToArmPreset(ArmPreset.FloorAlgae).repeatedly());
    //armXbox.a().whileTrue(new GoToArmPreset(ArmPreset.CoralStationFeed).repeatedly());
    //armXbox.x().whileTrue(Commands.run(()->armSubsystem.setArmConfigurationInOrder(ArmPreset.SlingshotAlgae, new ArmOrder(JointType.Shoulder, JointType.Telescopic, JointType.Wrist, 2, Units.inchesToMeters(3), 3)), armSubsystem));
    //armXbox.x().whileTrue(Commands.run(()->armSubsystem.setJointPosition(JointType.Shoulder, 90), armSubsystem));
    //armXbox.a().whileTrue(Commands.run(()->armSubsystem.setJointPosition(JointType.Shoulder, 45), armSubsystem));


    // armXbox.rightTrigger(.10).whileTrue(Commands.runEnd(()->intakeSubsystem.setIntakeVelocity(armXbox.getRightTriggerAxis()/7), ()->intakeSubsystem.setIntakeVelocity(0), armSubsystem));
    // armXbox.leftTrigger(.10).whileTrue(Commands.runEnd(()->intakeSubsystem.setIntakeVelocity(-armXbox.getLeftTriggerAxis()/7), ()->intakeSubsystem.setIntakeVelocity(0), armSubsystem));
    armXbox.rightTrigger(.5).onTrue(Commands.runOnce(()->intakeSubsystem.toggleIntake(0.185), armSubsystem));
    armXbox.leftTrigger(.5).onTrue(Commands.runOnce(()->intakeSubsystem.toggleOutake(-1), armSubsystem));
    armXbox.leftTrigger(.5).and(armXbox.rightTrigger(.5)).onTrue(Commands.runOnce(()->intakeSubsystem.turnOffIntake(), armSubsystem));
    armXbox.y().onTrue(Commands.runOnce(()-> {
      System.out.println("---------------------");
      System.out.println("Shoulder: "+armSubsystem.getJointPosition(JointType.Shoulder));
      System.out.println("Telescopic: "+Units.metersToInches(armSubsystem.getJointPosition(JointType.Telescopic)));
      System.out.println("Wrist: "+armSubsystem.getJointPosition(JointType.Wrist));
    }));

    swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    //armSubsystem.setDefaultCommand(armControllerCommand);
    if (Robot.isSimulation()) {
      driverXbox.a().onTrue(
          Commands.runOnce(() -> swerveSubsystem.swerveDrive.resetOdometry(new Pose2d(7.6, 1.178, new Rotation2d()))));
    }
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
