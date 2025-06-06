// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Utils.ArmOrder;
import frc.robot.Utils.ArmPreset;
import frc.robot.Utils.BargeTarget;
import frc.robot.Utils.CoralStationTarget;
import frc.robot.Utils.JointType;
import frc.robot.Utils.ProcessorTarget;
import frc.robot.Utils.ReefTarget;
import frc.robot.Utils.InputsManager.ForwardKinematicsInputsManager;
import frc.robot.commands.ArmControllerCommand;
import frc.robot.commands.AutoAlignAlgae;
import frc.robot.commands.GoToArmPreset;
import frc.robot.commands.GoToCoralStationGrooveBasedOnPoseEstimation;
import frc.robot.commands.GoToFieldTargetArmPreset;
import frc.robot.commands.GoToFieldTargetBasedOnPoseEstimation;
import frc.robot.commands.GoToProcessorBasedOnPoseEstimation;
import frc.robot.commands.GoToReefBasedOnPoseEstimation;
import frc.robot.commands.GoToReefFieldTargetBasedOnPoseEstimation;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OutakeCommand;
import frc.robot.commands.ZeroArm;
import frc.robot.field.FieldConstants;
import frc.robot.field.FieldConstants.CoralStation;
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
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
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
  // private final CommandXboxController armXbox = new CommandXboxController(OperatorConstants.ARM_CONTROLLER_PORT);
  private final CommandGenericHID buttonPad = new CommandGenericHID(OperatorConstants.BUTTON_PAD_PORT);
  // private final ForwardKinematicsInputsManager forwardKinematicsInputsManager = new ForwardKinematicsInputsManager(()->armXbox.getLeftY(), ()->-armXbox.getRightY(), ()->armXbox.getLeftX());
  // private final ArmControllerCommand armControllerCommand = new ArmControllerCommand(forwardKinematicsInputsManager);
  
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
      () -> -driverXbox.getLeftY(),
      () -> -driverXbox.getLeftX())
      .withControllerRotationAxis(() -> -driverXbox.getRightX())
      .deadband(OperatorConstants.SWERVE_DEADBAND)
      .scaleTranslation(0.3).scaleRotation(0.3)
      .allianceRelativeControl(false);

  public RobotContainer() {
    // Configure the trigger bindings
    //System.out.println(DriverStation.getAlliance().get());
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("ZeroArm", new ZeroArm());
    NamedCommands.registerCommand("ArmPresetLowAlgae", new GoToArmPreset(ArmPreset.LowAlgae));
    NamedCommands.registerCommand("ArmPresetHighAlgae", new GoToArmPreset(ArmPreset.HighAlgae));
    NamedCommands.registerCommand("ArmPresetProcessor", new GoToArmPreset(ArmPreset.Processor));
    NamedCommands.registerCommand("WaitHalfSecond", Commands.waitSeconds(0.5));
    NamedCommands.registerCommand("WaitFullSecond", Commands.waitSeconds(1));
    NamedCommands.registerCommand("WaitFullHalfSecond", Commands.waitSeconds(1.5));
    NamedCommands.registerCommand("CycleFront", new SequentialCommandGroup(
      swerveSubsystem.pathfindToFieldTarget(ReefTarget.FrontReef.Center, true),
      new GoToArmPreset(ArmPreset.LowAlgae),
      swerveSubsystem.getBaseAutonSequentialCommandGroup()));
    NamedCommands.registerCommand("CycleFrontLeft", new SequentialCommandGroup(
      swerveSubsystem.pathfindToFieldTarget(ReefTarget.FrontLeftReef.Center, true),
      new GoToArmPreset(ArmPreset.HighAlgae),
      swerveSubsystem.getBaseAutonSequentialCommandGroup()));
    NamedCommands.registerCommand("CycleFrontRight", new SequentialCommandGroup(
      swerveSubsystem.pathfindToFieldTarget(ReefTarget.FrontRightReef.Center, true),
      new GoToArmPreset(ArmPreset.HighAlgae),
      swerveSubsystem.getBaseAutonSequentialCommandGroup()));
    NamedCommands.registerCommand("CycleBack", new SequentialCommandGroup(
      swerveSubsystem.pathfindToFieldTarget(ReefTarget.BackReef.Center, true),
      new GoToArmPreset(ArmPreset.HighAlgae),
      swerveSubsystem.getBaseAutonSequentialCommandGroup()));
    NamedCommands.registerCommand("CycleBackRight", new SequentialCommandGroup(
      swerveSubsystem.pathfindToFieldTarget(ReefTarget.BackRightReef.Center, true),
      new GoToArmPreset(ArmPreset.LowAlgae),
      swerveSubsystem.getBaseAutonSequentialCommandGroup()));
    NamedCommands.registerCommand("CycleBackLeft", new SequentialCommandGroup(
      swerveSubsystem.pathfindToFieldTarget(ReefTarget.BackLeftReef.Center, true),
      new GoToArmPreset(ArmPreset.LowAlgae),
      swerveSubsystem.getBaseAutonSequentialCommandGroup()));
    

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Routine", autoChooser);
    configureBindings();
  }

  private void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = swerveSubsystem.driveFieldOriented(driveAngularVelocity);

    driverXbox.b().onTrue(Commands.runOnce(() -> swerveSubsystem.zeroFieldOrientedHeading(driveAngularVelocity), swerveSubsystem));
    
  
    driverXbox.rightBumper().whileTrue(new GoToFieldTargetBasedOnPoseEstimation(true));
    driverXbox.leftBumper().whileTrue(new GoToFieldTargetBasedOnPoseEstimation(false));
    //driverXbox.y().whileTrue(new ZeroArm().andThen(new GoToFieldTargetArmPreset().repeatedly()));
    driverXbox.y().whileTrue(new GoToReefBasedOnPoseEstimation(false));
    driverXbox.x().whileTrue(new ZeroArm().repeatedly());
    driverXbox.a().whileTrue(new ZeroArm().andThen(new GoToArmPreset(ArmPreset.FloorAlgae).repeatedly()));
    driverXbox.rightTrigger(.5).onTrue(Commands.runOnce(()->intakeSubsystem.toggleIntakeDefaultSpeed(), intakeSubsystem));
    driverXbox.leftTrigger(.5).onTrue(Commands.runOnce(()->intakeSubsystem.toggleOutakeDefaultSpeed(), armSubsystem));
    driverXbox.leftTrigger(.5).and(driverXbox.rightTrigger(.5)).onTrue(Commands.runOnce(()->intakeSubsystem.turnOffIntake(), armSubsystem));
    
    driverXbox.povDown().whileTrue(new GoToArmPreset(ArmPreset.LowAlgae).repeatedly());
    //driverXbox.povRight().whileTrue(new GoToArmPreset(ArmPreset.Processor).repeatedly());
    driverXbox.povRight().whileTrue(Commands.runOnce(()->intakeSubsystem.toggleOutakeSlowSpeed()));
    driverXbox.povUp().whileTrue(new GoToArmPreset(ArmPreset.HighAlgae).repeatedly());
    // driverXbox.povLeft().whileTrue(Commands.runOnce(()->armSubsystem.setArmConfigurationInOrder(ArmPreset.BargeAlgae, new ArmOrder(JointType.Shoulder, JointType.Wrist, JointType.Telescopic, 3, 10, Units.inchesToMeters(2))), armSubsystem).repeatedly());
    driverXbox.povLeft().whileTrue(Commands.runOnce(()->armSubsystem.setArmConfigurationOptimally(ArmPreset.BargeAlgae.armConfiguration)).repeatedly());
    // driverXbox.povLeft().whileTrue(new GoToReefFieldTargetBasedOnPoseEstimation(true, true));
    // driverXbox.povRight().whileTrue(new GoToReefFieldTargetBasedOnPoseEstimation(false, true));

    buttonPad.button(1).whileTrue(swerveSubsystem.pathfindToFieldTarget(ReefTarget.FrontLeftReef.Center, true));
    buttonPad.button(1).onTrue(Commands.runOnce(()->swerveSubsystem.currentFieldTarget = ReefTarget.FrontLeftReef.Center));

    buttonPad.button(2).whileTrue(swerveSubsystem.pathfindToFieldTarget(ReefTarget.BackLeftReef.Center, true));
    buttonPad.button(2).onTrue(Commands.runOnce(()->swerveSubsystem.currentFieldTarget = ReefTarget.BackLeftReef.Center));

    buttonPad.button(3).whileTrue(swerveSubsystem.pathfindToFieldTarget(ReefTarget.BackReef.Center, true));
    buttonPad.button(3).onTrue(Commands.runOnce(()->swerveSubsystem.currentFieldTarget = ReefTarget.BackReef.Center));

    buttonPad.button(4).whileTrue(swerveSubsystem.pathfindToFieldTarget(ReefTarget.FrontReef.Center, true));
    buttonPad.button(4).onTrue(Commands.runOnce(()->swerveSubsystem.currentFieldTarget = ReefTarget.FrontReef.Center));

    buttonPad.button(5).whileTrue(swerveSubsystem.pathfindToFieldTarget(CoralStationTarget.Right, true));
    buttonPad.button(5).onTrue(Commands.runOnce(()->swerveSubsystem.currentFieldTarget = CoralStationTarget.Right));

    buttonPad.button(6).whileTrue(swerveSubsystem.pathfindToFieldTarget(ReefTarget.FrontRightReef.Center, true));
    buttonPad.button(6).onTrue(Commands.runOnce(()->swerveSubsystem.currentFieldTarget = ReefTarget.FrontRightReef.Center));

    buttonPad.button(7).whileTrue(swerveSubsystem.pathfindToFieldTarget(CoralStationTarget.Left, true));
    buttonPad.button(7).onTrue(Commands.runOnce(()->swerveSubsystem.currentFieldTarget = CoralStationTarget.Left));

    buttonPad.button(8).whileTrue(swerveSubsystem.pathfindToFieldTarget(ReefTarget.BackRightReef.Center, true));
    buttonPad.button(8).onTrue(Commands.runOnce(()->swerveSubsystem.currentFieldTarget = ReefTarget.BackRightReef.Center));

    buttonPad.button(10).whileTrue(swerveSubsystem.pathfindToFieldTarget(ProcessorTarget.Processor,true));
    buttonPad.button(10).onTrue(Commands.runOnce(()->swerveSubsystem.currentFieldTarget = ProcessorTarget.Processor));

    // buttonPad.button(11).whileTrue(
    //   Commands.runOnce(()->{swerveSubsystem.setCurrentFieldTarget(FieldConstants.getCoralTargetFromReefTarget(swerveSubsystem.getCurrentFieldTarget(), true)); System.out.println(swerveSubsystem.currentFieldTarget);})
    //   .andThen(swerveSubsystem.pathfindToFieldTarget(swerveSubsystem.currentFieldTarget, true)));

    // buttonPad.button(12).whileTrue(
    //   Commands.runOnce(()->{swerveSubsystem.setCurrentFieldTarget(FieldConstants.getCoralTargetFromReefTarget(swerveSubsystem.getCurrentFieldTarget(), false)); System.out.println(swerveSubsystem.currentFieldTarget);})
    //   .andThen(swerveSubsystem.pathfindToFieldTarget(swerveSubsystem.currentFieldTarget, true)));

    // buttonPad.button(11).whileTrue(new GoToReefFieldTargetBasedOnPoseEstimation(true, false));

    // buttonPad.button(12).whileTrue(new GoToReefFieldTargetBasedOnPoseEstimation(false, false));

    // armXbox.a().whileTrue(new GoToReefFieldTargetBasedOnPoseEstimation(true, false));
    // armXbox.b().whileTrue(new GoToReefFieldTargetBasedOnPoseEstimation(false, false));

    buttonPad.button(9).whileTrue(swerveSubsystem.removeAlgaeFromCurrentReefTarget());
    // buttonPad.povDown().whileTrue(new GoToReefFieldTargetBasedOnPoseEstimation(false, false));

    buttonPad.axisLessThan(0, -0.9).whileTrue(swerveSubsystem.pathfindToFieldTarget(BargeTarget.Center,false));
    buttonPad.axisLessThan(0, -0.9).whileTrue(Commands.runOnce(()->swerveSubsystem.currentFieldTarget = BargeTarget.Center));

    buttonPad.axisGreaterThan(0, 0.9).whileTrue(new AutoAlignAlgae());

    buttonPad.axisGreaterThan(1, 0.9).whileTrue(swerveSubsystem.pathfindToFieldTarget(BargeTarget.Left,false));
    buttonPad.axisGreaterThan(1, 0.9).whileTrue(Commands.runOnce(()->swerveSubsystem.currentFieldTarget = BargeTarget.Left));

    buttonPad.axisLessThan(1, -0.9).whileTrue(swerveSubsystem.pathfindToFieldTarget(BargeTarget.Right,false));
    buttonPad.axisLessThan(1, -0.9).whileTrue(Commands.runOnce(()->swerveSubsystem.currentFieldTarget = BargeTarget.Right));


    //armXbox.a().whileTrue(new GoToArmPreset(ArmPreset.CoralStationFeed).repeatedly());
    //armXbox.x().whileTrue(Commands.run(()->armSubsystem.setArmConfigurationInOrder(ArmPreset.SlingshotAlgae, new ArmOrder(JointType.Shoulder, JointType.Telescopic, JointType.Wrist, 2, Units.inchesToMeters(3), 3)), armSubsystem));
    // armXbox.x().whileTrue(Commands.run(()->armSubsystem.setJointPosition(JointType.Wrist, 90), armSubsystem));
    // armXbox.a().whileTrue(Commands.run(()->armSubsystem.setJointPosition(JointType.Wrist, 0), armSubsystem));


    // armXbox.rightTrigger(.10).whileTrue(Commands.runEnd(()->intakeSubsystem.setIntakeVelocity(armXbox.getRightTriggerAxis()/7), ()->intakeSubsystem.setIntakeVelocity(0), armSubsystem));
    // armXbox.leftTrigger(.10).whileTrue(Commands.runEnd(()->intakeSubsystem.setIntakeVelocity(-armXbox.getLeftTriggerAxis()/7), ()->intakeSubsystem.setIntakeVelocity(0), armSubsystem));
    
    // driverXbox.y().onTrue(Commands.runOnce(()-> {
    //   System.out.println("---------------------");
    //   System.out.println("Shoulder: "+armSubsystem.getJointPosition(JointType.Shoulder));
    //   System.out.println("Telescopic: "+Units.metersToInches(armSubsystem.getJointPosition(JointType.Telescopic)));
    //   System.out.println("Wrist: "+armSubsystem.getJointPosition(JointType.Wrist));
    // }));

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
