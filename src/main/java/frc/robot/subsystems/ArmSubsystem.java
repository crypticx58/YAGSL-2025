package frc.robot.subsystems;


import com.fasterxml.jackson.databind.JsonSerializable.Base;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import static java.util.Map.entry;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Utils.JointType;

public class ArmSubsystem extends SubsystemBase {
    private final static ArmSubsystem INSTANCE = new ArmSubsystem();

    @SuppressWarnings("WeakerAccess")
    public static ArmSubsystem getInstance() {
        return INSTANCE;
    }

    private final SparkMax ShoulderJoint = new SparkMax(JointType.Shoulder.ID, MotorType.kBrushless);
    private final SparkMax TelescopicJoint = new SparkMax(JointType.Telescopic.ID, MotorType.kBrushless);
    private final SparkMax WristJoint = new SparkMax(JointType.Wrist.ID, MotorType.kBrushless);
    private final SparkMax IntakeJoint = new SparkMax(JointType.Intake.ID, MotorType.kBrushless);

    private final PhotonCamera topCamera = new  PhotonCamera("TopCamera");
    private final PhotonCamera sideCamera = new  PhotonCamera("SideCamera");
    
    private final Map<JointType, SparkMax> Joints = Map.of(JointType.Shoulder, ShoulderJoint, JointType.Telescopic, TelescopicJoint, JointType.Wrist, WristJoint, JointType.Intake, IntakeJoint);
    private ArmSubsystem() {
        configureConversionFactors();
        configureOffsets();
        // configurePIDs();
        PhotonCamera.setVersionCheckEnabled(false);
    }
    private void configureConversionFactors(){
        // Degrees
        SparkMaxConfig ShoulderJointConfig = new SparkMaxConfig();
        SparkMaxConfig TelescopicJointConfig = new SparkMaxConfig();
        SparkMaxConfig WristJointConfig = new SparkMaxConfig();
        SparkMaxConfig IntakeJointConfig = new SparkMaxConfig();

        ShoulderJointConfig.inverted(false).idleMode(IdleMode.kBrake);
        TelescopicJointConfig.inverted(true).idleMode(IdleMode.kBrake);
        WristJointConfig.inverted(false).idleMode(IdleMode.kBrake);
        IntakeJointConfig.inverted(false).idleMode(IdleMode.kBrake);


        // Mechanism rotations in degrees or Meters
        ShoulderJointConfig.encoder.positionConversionFactor(JointType.Shoulder.gearRatio*360);
        TelescopicJointConfig.encoder.positionConversionFactor(JointType.Telescopic.gearRatio*1);//Figure out the factor
        WristJointConfig.encoder.positionConversionFactor(JointType.Wrist.gearRatio*360);
        //IntakeJointConfig.encoder.positionConversionFactor(Constants.ArmConstants.IntakeJointGearRatio*360);

        // Degrees per second, DPS
        ShoulderJointConfig.encoder.velocityConversionFactor(JointType.Shoulder.gearRatio*360*60);
        TelescopicJointConfig.encoder.velocityConversionFactor(JointType.Telescopic.gearRatio*1*60);//Figure out the factor, MPS, Meters Per Sec
        WristJointConfig.encoder.velocityConversionFactor(JointType.Wrist.gearRatio*360*60);
        //IntakeJointConfig.encoder.velocityConversionFactor(Constants.ArmConstants.WristJointGearRatio*360*60);

        ShoulderJointConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.04, 0, 0);
        TelescopicJointConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.04, 0, 0);
        WristJointConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.04, 0, 0);
        //IntakeJointConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.04, 0, 0);

        ShoulderJoint.configure(ShoulderJointConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        TelescopicJoint.configure(TelescopicJointConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        WristJoint.configure(WristJointConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        IntakeJoint.configure(IntakeJointConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
    private void configureOffsets(){
        ShoulderJoint.getEncoder().setPosition(0); // figure out (Degrees)
        TelescopicJoint.getEncoder().setPosition(0); // figure out (Meters)
        WristJoint.getEncoder().setPosition(0); // figure out (Degrees)
    }
    public void setJointPosition(JointType jointType, double jointPosition){ // Degrees or Inches
        Joints.get(jointType).getClosedLoopController().setReference(jointPosition, ControlType.kPosition);
    }
    public void setJointSpeed(JointType jointType, double jointSpeed){
        if (jointType == JointType.Intake){
            Joints.get(jointType).set(jointSpeed);
            return;
        }
        Joints.get(jointType).getClosedLoopController().setReference(jointSpeed, ControlType.kVelocity);
    }
    public double getJointPosition(JointType jointType){ // Degrees or Inches
        return Joints.get(jointType).getEncoder().getPosition();
    }
    public void stopJointsInPlace(){
        for (SparkMax joint : Joints.values()){
            joint.getClosedLoopController().setReference(0, ControlType.kVelocity);
        }
    }
    public Pose3d getCoralPose3dRelativeToRobot(Pose3d robotPose3d){
        double TelescopicLength = getJointPosition(JointType.Telescopic);
        double ShoulderAngle = getJointPosition(JointType.Shoulder);
        double WristAngle = getJointPosition(JointType.Wrist);
        Pose3d ShoulderJointPose3d = robotPose3d.plus(ArmConstants.RobotToShoulderJointTransform);
        Transform3d ShoulderJointToWristJointTransform3d = new Transform3d(
            new Translation3d(TelescopicLength*Math.cos(Units.degreesToRadians(ShoulderAngle)),0,TelescopicLength*Math.sin(Units.degreesToRadians(ShoulderAngle))),
            new Rotation3d());
        
        Pose3d WristJointPos3d = ShoulderJointPose3d.plus(ShoulderJointToWristJointTransform3d);
        //This goes from the wrist to the attachment point of the wrist and intake. It also gains the wrists orientation, making it equivalent to the normal vector of the attachment point.
        Transform3d wristJointToIntakeTransform3d = new Transform3d(
            new Translation3d(ArmConstants.WristJointLengthMeters*Math.cos(Units.degreesToRadians(WristAngle)),0,ArmConstants.WristJointLengthMeters*Math.cos(Units.degreesToRadians(WristAngle))),
            new Rotation3d(0,-Units.degreesToRadians(WristAngle),0));
        Pose3d intakeAttachmentPointPose3d = WristJointPos3d.plus(wristJointToIntakeTransform3d);
        Pose3d coralPose3d = intakeAttachmentPointPose3d.plus(ArmConstants.IntakeToCoralTransform);
        return coralPose3d;
    }
    // Gets the angle of the object with respect to the base from the top camera
//     public Pose3d get3dPoseOfColoredObjectRelativeToBase(){
//         PhotonPipelineResult topResult = topCamera.getLatestResult();
//         PhotonPipelineResult sideResult = sideCamera.getLatestResult();
//         PhotonTrackedTarget topTarget = topResult.getBestTarget();
//         PhotonTrackedTarget sideTarget = topResult.getBestTarget();

//         Pose3d targetPoseRelativeToTopCamera = ArmConstants.BasePose3d.plus(VisionConstants.BaseToTopCameraTransform).plus(topTarget.getBestCameraToTarget());
//         Pose3d targetPoseRelativeToSideCamera = ArmConstants.BasePose3d.plus(VisionConstants.BaseToSideCameraTransform).plus(sideTarget.getBestCameraToTarget());

    //}
}
