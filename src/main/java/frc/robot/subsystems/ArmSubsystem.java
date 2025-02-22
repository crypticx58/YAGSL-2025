package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;

import frc.robot.Constants.ArmConstants;
import frc.robot.utils.ArmConfiguration;
import frc.robot.utils.JointType;

public class ArmSubsystem extends SubsystemBase {
    private static ArmSubsystem INSTANCE;

    @SuppressWarnings("WeakerAccess")
    public static ArmSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new ArmSubsystem();
        }
        return INSTANCE;
    }

    private final SparkMax ShoulderJoint = new SparkMax(JointType.Shoulder.ID, MotorType.kBrushless);
    private final SparkMax TelescopicJoint = new SparkMax(JointType.Telescopic.ID, MotorType.kBrushless);
    private final SparkMax WristJoint = new SparkMax(JointType.Wrist.ID, MotorType.kBrushless);

    private final Map<JointType, SparkMax> Joints = Map.of(JointType.Shoulder, ShoulderJoint, JointType.Telescopic,
            TelescopicJoint, JointType.Wrist, WristJoint);

    private ArmSubsystem() {
        configureJoints();
        configureOffsets();
    }
    @Override
    public void periodic(){
    }

    private void configureJoints() {
        SparkMaxConfig ShoulderJointConfig = new SparkMaxConfig();
        SparkMaxConfig TelescopicJointConfig = new SparkMaxConfig();
        SparkMaxConfig WristJointConfig = new SparkMaxConfig();

        ShoulderJointConfig.inverted(false).idleMode(IdleMode.kBrake);
        TelescopicJointConfig.inverted(true).idleMode(IdleMode.kBrake);
        WristJointConfig.inverted(false).idleMode(IdleMode.kBrake);

        ShoulderJointConfig.encoder.positionConversionFactor(2.2359); // Degrees
        TelescopicJointConfig.encoder.positionConversionFactor(0.0364823762189); // METERS
        WristJointConfig.encoder.positionConversionFactor(8); // Degrees

        ShoulderJointConfig.closedLoopRampRate(0.3);
        TelescopicJointConfig.closedLoopRampRate(0.3);
        WristJointConfig.closedLoopRampRate(0.3);

        ShoulderJointConfig.encoder.velocityConversionFactor(0.04); // Degrees PER SEC
        TelescopicJointConfig.encoder.velocityConversionFactor(0.000608039603648); // METERS per sec
        WristJointConfig.encoder.velocityConversionFactor(0.133333333333); // Degrees per sec

        ShoulderJointConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.0045, 0, 0.000, ClosedLoopSlot.kSlot0).outputRange(-1, 1).velocityFF(1/473, ClosedLoopSlot.kSlot0);
        ShoulderJointConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.030, 0, 0.000, ClosedLoopSlot.kSlot1).outputRange(-1, 1);

        TelescopicJointConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.675, 0, 0, ClosedLoopSlot.kSlot0).outputRange(-1, 1).velocityFF(1/473, ClosedLoopSlot.kSlot0);
        TelescopicJointConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(2.4, 0, 0.000, ClosedLoopSlot.kSlot1).outputRange(-1, 1);

        WristJointConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.0020, 0, 0.000, ClosedLoopSlot.kSlot0).outputRange(-1, 1).velocityFF(1/473, ClosedLoopSlot.kSlot0);
        WristJointConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.01, 0, 0.000, ClosedLoopSlot.kSlot1).outputRange(-1, 1);

        ShoulderJoint.configure(ShoulderJointConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        TelescopicJoint.configure(TelescopicJointConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        WristJoint.configure(WristJointConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureOffsets() {
        ShoulderJoint.getEncoder().setPosition(-1); 
        TelescopicJoint.getEncoder().setPosition(Units.inchesToMeters(23.5)); 
        WristJoint.getEncoder().setPosition(-90);
    }
    public void setPercentage(JointType jointType, double percentage){
        Joints.get(jointType).set(percentage);
    }
    public void setJointPosition(JointType jointType, double jointPosition) {
        double arbitraryFeedForward = 0;
        if (jointType == JointType.Shoulder || jointType == JointType.Wrist){
            arbitraryFeedForward = 0.25*Math.cos(Units.degreesToRadians(getJointPosition(jointType)));
        }
        if (!isPositionSetpointValid(jointType, jointPosition)){
            Joints.get(jointType).getClosedLoopController().setReference(getJointPosition(jointType), ControlType.kPosition, ClosedLoopSlot.kSlot1, arbitraryFeedForward);
            return;
        }
        Joints.get(jointType).getClosedLoopController().setReference(jointPosition, ControlType.kPosition, ClosedLoopSlot.kSlot1, arbitraryFeedForward);
    }

    public void setJointVelocity(JointType jointType, double jointVelocity) {
        double arbitraryFeedForward = 0;
        if (jointType == JointType.Shoulder || jointType == JointType.Wrist){
            arbitraryFeedForward = getJointPosition(jointType)<90 ? 0.25*Math.cos(Units.degreesToRadians(getJointPosition(jointType))):0;
        } else if (jointType == JointType.Telescopic){
            arbitraryFeedForward = 0.25*Math.sin(Units.degreesToRadians(getJointPosition(jointType)));
        }
        if (!isVelocitySetpointValid(jointType, jointVelocity)){
            Joints.get(jointType).getClosedLoopController().setReference(0, ControlType.kVelocity, ClosedLoopSlot.kSlot0, arbitraryFeedForward);
            return;
        }
        Joints.get(jointType).getClosedLoopController().setReference(jointVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0, arbitraryFeedForward);
    }
    public boolean isJointWithinRange(JointType jointType){
        double jointPosition = getJointPosition(jointType);
        return (jointPosition >= jointType.lowerBound) && (jointPosition <= jointType.upperBound);
    }
    public boolean isPositionSetpointValid(JointType jointType, double setpoint){
        if (setpoint < jointType.lowerBound) return false;
        if (setpoint > jointType.upperBound) return false;
        return true;
    }
    public boolean isVelocitySetpointValid(JointType jointType, double setpoint){
        if (getJointPosition(jointType) <= jointType.lowerBound && setpoint < 0) return false;
        if (getJointPosition(jointType) >= jointType.upperBound && setpoint > 0) return false;
        return true;
    }
    public double getJointPosition(JointType jointType) { // Degrees or Inches
        return Joints.get(jointType).getEncoder().getPosition();
    }
    public boolean isJointAtSetpoint(JointType jointType, double setpoint, double tolerance){
        return (Math.abs(getJointPosition(jointType)-setpoint)<=tolerance);
    }
    public boolean isArmAtDesiredConfiguration(ArmConfiguration armConfiguration, double shoulderTolerance, double telescopicTolerance, double WristTolerance){
        return isJointAtSetpoint(JointType.Shoulder, armConfiguration.ShoulderPosition, shoulderTolerance) &&
         isJointAtSetpoint(JointType.Telescopic, armConfiguration.TelescopicPosition, telescopicTolerance) &&
         isJointAtSetpoint(JointType.Wrist, armConfiguration.WristPosition, WristTolerance);
    }
    public double getJointVelocity(JointType jointType) {
        return Joints.get(jointType).getEncoder().getVelocity();
    }
    public void setJointEncoderPosition(JointType jointType, double position){
        Joints.get(jointType).getEncoder().setPosition(position);
    }
    public void stopJointsInPlace() {
        for (SparkMax joint : Joints.values()) {
            joint.getClosedLoopController().setReference(0, ControlType.kVelocity);
        }
    }
    public void setArmConfiguration(ArmConfiguration armConfiguration){
        setJointPosition(JointType.Shoulder, armConfiguration.ShoulderPosition);
        setJointPosition(JointType.Telescopic, armConfiguration.TelescopicPosition);
        setJointPosition(JointType.Wrist, armConfiguration.WristPosition);
    }

    //Has to be called mulitiple times/periodically. Probably should NOT use to zero out the arm
    public void setArmConfigurationOptimally(ArmConfiguration armConfiguration){
        if (isJointAtSetpoint(JointType.Shoulder, armConfiguration.ShoulderPosition, 2.5)){
            setJointPosition(JointType.Shoulder, armConfiguration.ShoulderPosition);
            setJointPosition(JointType.Telescopic, armConfiguration.TelescopicPosition);
            setJointPosition(JointType.Wrist, armConfiguration.WristPosition);
        } else if (getJointPosition(JointType.Shoulder) >= 25) { // We start moving the wrist
            setJointPosition(JointType.Shoulder, armConfiguration.ShoulderPosition);
            setJointPosition(JointType.Wrist, armConfiguration.WristPosition);
        } else {
            setJointPosition(JointType.Shoulder, armConfiguration.ShoulderPosition);
        }
    }
    public Pose3d getCoralPose3dRelativeToRobot(Pose3d robotPose3d) {
        double TelescopicLength = getJointPosition(JointType.Telescopic);
        double ShoulderAngle = getJointPosition(JointType.Shoulder);
        double WristAngle = getJointPosition(JointType.Wrist);
        Pose3d ShoulderJointPose3d = robotPose3d.plus(ArmConstants.RobotToShoulderJointTransform);
        Transform3d ShoulderJointToWristJointTransform3d = new Transform3d(
                new Translation3d(TelescopicLength * Math.cos(Units.degreesToRadians(ShoulderAngle)), 0,
                        TelescopicLength * Math.sin(Units.degreesToRadians(ShoulderAngle))),
                new Rotation3d());

        Pose3d WristJointPos3d = ShoulderJointPose3d.plus(ShoulderJointToWristJointTransform3d);
        // This goes from the wrist to the attachment point of the wrist and intake. It
        // also gains the wrists orientation, making it equivalent to the normal vector
        // of the attachment point.
        Transform3d wristJointToIntakeTransform3d = new Transform3d(
                new Translation3d(ArmConstants.WristJointLengthMeters * Math.cos(Units.degreesToRadians(WristAngle)), 0,
                        ArmConstants.WristJointLengthMeters * Math.cos(Units.degreesToRadians(WristAngle))),
                new Rotation3d(0, -Units.degreesToRadians(WristAngle), 0));
        Pose3d intakeAttachmentPointPose3d = WristJointPos3d.plus(wristJointToIntakeTransform3d);
        Pose3d coralPose3d = intakeAttachmentPointPose3d.plus(ArmConstants.IntakeToCoralTransform);
        return coralPose3d;
    }
}
