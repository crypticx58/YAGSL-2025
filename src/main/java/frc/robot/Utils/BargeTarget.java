package frc.robot.Utils;

import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import frc.robot.field.FieldConstants;
import frc.robot.field.FieldConstants.AprilTagIDs;

public enum BargeTarget implements FieldTarget {
    Center(AprilTagIDs.getAllianceBargeId(), FieldConstants.aprilTagFieldLayout.getTagPose(AprilTagIDs.getAllianceBargeId()).get()){
        public int getApriltagId() {return ApriltagId;}
        public Pose3d getTargetPose() {return TargetPose;}
    },
    Left(AprilTagIDs.getAllianceBargeId(), FieldConstants.aprilTagFieldLayout.getTagPose(AprilTagIDs.getAllianceBargeId()).get().plus(FieldConstants.Barge.LeftBargeOffset)){
        public int getApriltagId() {return ApriltagId;}
        public Pose3d getTargetPose() {return TargetPose;}
    },
    Right(AprilTagIDs.getAllianceBargeId(), FieldConstants.aprilTagFieldLayout.getTagPose(AprilTagIDs.getAllianceBargeId()).get().plus(FieldConstants.Barge.RightBargeOffset)){
        public int getApriltagId() {return ApriltagId;}
        public Pose3d getTargetPose() {return TargetPose;}
    };
    public final int ApriltagId;
    public final Pose3d TargetPose;
    private BargeTarget(int ApriltagId, Pose3d TargetPose){
        this.ApriltagId = ApriltagId;
        this.TargetPose = TargetPose;
    }
}
