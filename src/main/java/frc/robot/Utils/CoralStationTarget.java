package frc.robot.Utils;

import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import frc.robot.field.FieldConstants;
import frc.robot.field.FieldConstants.AprilTagIDs;

public enum CoralStationTarget implements FieldTarget {
    Left(AprilTagIDs.getLeftAllianceCoralStationId(), FieldConstants.aprilTagFieldLayout.getTagPose(AprilTagIDs.getLeftAllianceCoralStationId()).get()){
        public int getApriltagId() {return ApriltagId;}
        public Pose3d getTargetPose() {return TargetPose;}
    },
    Right(AprilTagIDs.getRightAllianceCoralStationId(), FieldConstants.aprilTagFieldLayout.getTagPose(AprilTagIDs.getRightAllianceCoralStationId()).get()){
        public int getApriltagId() {return ApriltagId;}
        public Pose3d getTargetPose() {return TargetPose;}
    };
    public final int ApriltagId;
    public final Pose3d TargetPose;
    private CoralStationTarget(int ApriltagId, Pose3d TargetPose){
        this.ApriltagId = ApriltagId;
        this.TargetPose = TargetPose;
    }
}
