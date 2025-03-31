package frc.robot.Utils;

import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import frc.robot.field.FieldConstants;
import frc.robot.field.FieldConstants.AprilTagIDs;

public enum ProcessorTarget implements FieldTarget {
    Processor(AprilTagIDs.getAllianceProcessorId(), FieldConstants.aprilTagFieldLayout.getTagPose(AprilTagIDs.getAllianceProcessorId()).get()){
        public int getApriltagId() {return ApriltagId;}
        public Pose3d getTargetPose() {return TargetPose;}
    };
    public final int ApriltagId;
    public final Pose3d TargetPose;
    private ProcessorTarget(int ApriltagId, Pose3d TargetPose){
        this.ApriltagId = ApriltagId;
        this.TargetPose = TargetPose;
    }
}
