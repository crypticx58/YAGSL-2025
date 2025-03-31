package frc.robot.Utils;

import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import frc.robot.field.FieldConstants;
import frc.robot.field.FieldConstants.AprilTagIDs;

public class ReefTarget{
    public enum FrontReef implements FieldTarget {
        Center(AprilTagIDs.getAllianceFrontReefId(), FieldConstants.aprilTagFieldLayout.getTagPose(AprilTagIDs.getAllianceFrontReefId()).get()){
            public int getApriltagId() {return ApriltagId;}
            public Pose3d getTargetPose() {return TargetPose;}
        },
        Left(AprilTagIDs.getAllianceFrontReefId(), FieldConstants.aprilTagFieldLayout.getTagPose(AprilTagIDs.getAllianceFrontReefId()).get().plus(FieldConstants.Reef.LeftOffsetTransformFromCenterMeters)){
            public int getApriltagId() {return ApriltagId;}
            public Pose3d getTargetPose() {return TargetPose;}
        },
        Right(AprilTagIDs.getAllianceFrontReefId(), FieldConstants.aprilTagFieldLayout.getTagPose(AprilTagIDs.getAllianceFrontReefId()).get().plus(FieldConstants.Reef.RightOffsetTransformFromCenterMeters)){
            public int getApriltagId() {return ApriltagId;}
            public Pose3d getTargetPose() {return TargetPose;}
        };
        public final int ApriltagId;
        public final Pose3d TargetPose;
        private FrontReef(int ApriltagId, Pose3d TargetPose){
            this.ApriltagId = ApriltagId;
            this.TargetPose = TargetPose;
        }
    }
    public enum FrontLeftReef implements FieldTarget {
        Center(AprilTagIDs.getAllianceFrontLeftReefId(), FieldConstants.aprilTagFieldLayout.getTagPose(AprilTagIDs.getAllianceFrontLeftReefId()).get()){
            public int getApriltagId() {return ApriltagId;}
            public Pose3d getTargetPose() {return TargetPose;}
        },
        Left(AprilTagIDs.getAllianceFrontLeftReefId(), FieldConstants.aprilTagFieldLayout.getTagPose(AprilTagIDs.getAllianceFrontLeftReefId()).get().plus(FieldConstants.Reef.LeftOffsetTransformFromCenterMeters)){
            public int getApriltagId() {return ApriltagId;}
            public Pose3d getTargetPose() {return TargetPose;}
        },
        Right(AprilTagIDs.getAllianceFrontLeftReefId(), FieldConstants.aprilTagFieldLayout.getTagPose(AprilTagIDs.getAllianceFrontLeftReefId()).get().plus(FieldConstants.Reef.RightOffsetTransformFromCenterMeters)){
            public int getApriltagId() {return ApriltagId;}
            public Pose3d getTargetPose() {return TargetPose;}
        };
        public final int ApriltagId;
        public final Pose3d TargetPose;
        private FrontLeftReef(int ApriltagId, Pose3d TargetPose){
            this.ApriltagId = ApriltagId;
            this.TargetPose = TargetPose;
        }
    }
    public enum FrontRightReef implements FieldTarget {
        Center(AprilTagIDs.getAllianceFrontRightReefId(), FieldConstants.aprilTagFieldLayout.getTagPose(AprilTagIDs.getAllianceFrontRightReefId()).get()){
            public int getApriltagId() {return ApriltagId;}
            public Pose3d getTargetPose() {return TargetPose;}
        },
        Left(AprilTagIDs.getAllianceFrontRightReefId(), FieldConstants.aprilTagFieldLayout.getTagPose(AprilTagIDs.getAllianceFrontRightReefId()).get().plus(FieldConstants.Reef.LeftOffsetTransformFromCenterMeters)){
            public int getApriltagId() {return ApriltagId;}
            public Pose3d getTargetPose() {return TargetPose;}
        },
        Right(AprilTagIDs.getAllianceFrontRightReefId(), FieldConstants.aprilTagFieldLayout.getTagPose(AprilTagIDs.getAllianceFrontRightReefId()).get().plus(FieldConstants.Reef.RightOffsetTransformFromCenterMeters)){
            public int getApriltagId() {return ApriltagId;}
            public Pose3d getTargetPose() {return TargetPose;}
        };
        public final int ApriltagId;
        public final Pose3d TargetPose;
        private FrontRightReef(int ApriltagId, Pose3d TargetPose){
            this.ApriltagId = ApriltagId;
            this.TargetPose = TargetPose;
        }
    }
    public enum BackReef implements FieldTarget {
        Center(AprilTagIDs.getAllianceBackReefId(), FieldConstants.aprilTagFieldLayout.getTagPose(AprilTagIDs.getAllianceBackReefId()).get()){
            public int getApriltagId() {return ApriltagId;}
            public Pose3d getTargetPose() {return TargetPose;}
        },
        Left(AprilTagIDs.getAllianceBackReefId(), FieldConstants.aprilTagFieldLayout.getTagPose(AprilTagIDs.getAllianceBackReefId()).get().plus(FieldConstants.Reef.LeftOffsetTransformFromCenterMeters)){
            public int getApriltagId() {return ApriltagId;}
            public Pose3d getTargetPose() {return TargetPose;}
        },
        Right(AprilTagIDs.getAllianceBackReefId(), FieldConstants.aprilTagFieldLayout.getTagPose(AprilTagIDs.getAllianceBackReefId()).get().plus(FieldConstants.Reef.RightOffsetTransformFromCenterMeters)){
            public int getApriltagId() {return ApriltagId;}
            public Pose3d getTargetPose() {return TargetPose;}
        };
        public final int ApriltagId;
        public final Pose3d TargetPose;
        private BackReef(int ApriltagId, Pose3d TargetPose){
            this.ApriltagId = ApriltagId;
            this.TargetPose = TargetPose;
        }
    }
    public enum BackLeftReef implements FieldTarget {
        Center(AprilTagIDs.getAllianceBackLeftReefId(), FieldConstants.aprilTagFieldLayout.getTagPose(AprilTagIDs.getAllianceBackLeftReefId()).get()){
            public int getApriltagId() {return ApriltagId;}
            public Pose3d getTargetPose() {return TargetPose;}
        },
        Left(AprilTagIDs.getAllianceBackLeftReefId(), FieldConstants.aprilTagFieldLayout.getTagPose(AprilTagIDs.getAllianceBackLeftReefId()).get().plus(FieldConstants.Reef.LeftOffsetTransformFromCenterMeters)){
            public int getApriltagId() {return ApriltagId;}
            public Pose3d getTargetPose() {return TargetPose;}
        },
        Right(AprilTagIDs.getAllianceBackLeftReefId(), FieldConstants.aprilTagFieldLayout.getTagPose(AprilTagIDs.getAllianceBackLeftReefId()).get().plus(FieldConstants.Reef.RightOffsetTransformFromCenterMeters)){
            public int getApriltagId() {return ApriltagId;}
            public Pose3d getTargetPose() {return TargetPose;}
        };
        public final int ApriltagId;
        public final Pose3d TargetPose;
        private BackLeftReef(int ApriltagId, Pose3d TargetPose){
            this.ApriltagId = ApriltagId;
            this.TargetPose = TargetPose;
        }
    }
    public enum BackRightReef implements FieldTarget {
        Center(AprilTagIDs.getAllianceBackRightReefId(), FieldConstants.aprilTagFieldLayout.getTagPose(AprilTagIDs.getAllianceBackRightReefId()).get()){
            public int getApriltagId() {return ApriltagId;}
            public Pose3d getTargetPose() {return TargetPose;}
        },
        Left(AprilTagIDs.getAllianceBackRightReefId(), FieldConstants.aprilTagFieldLayout.getTagPose(AprilTagIDs.getAllianceBackRightReefId()).get().plus(FieldConstants.Reef.LeftOffsetTransformFromCenterMeters)){
            public int getApriltagId() {return ApriltagId;}
            public Pose3d getTargetPose() {return TargetPose;}
        },
        Right(AprilTagIDs.getAllianceBackRightReefId(), FieldConstants.aprilTagFieldLayout.getTagPose(AprilTagIDs.getAllianceBackRightReefId()).get().plus(FieldConstants.Reef.RightOffsetTransformFromCenterMeters)){
            public int getApriltagId() {return ApriltagId;}
            public Pose3d getTargetPose() {return TargetPose;}
        };    
        public final int ApriltagId;
        public final Pose3d TargetPose;
        private BackRightReef(int ApriltagId, Pose3d TargetPose){
            this.ApriltagId = ApriltagId;
            this.TargetPose = TargetPose;
        }
    }
    public static boolean isReefTarget(FieldTarget fieldTarget){
        return isCoralReefTarget(fieldTarget) || isAlgaeReefTarget(fieldTarget);
    }
    public static boolean isCoralReefTarget(FieldTarget fieldTarget){
        return (fieldTarget == FrontReef.Left || fieldTarget == FrontLeftReef.Left || fieldTarget == FrontRightReef.Left || fieldTarget == BackReef.Left || fieldTarget == BackLeftReef.Left || fieldTarget == BackRightReef.Left ||
        fieldTarget == FrontReef.Right || fieldTarget == FrontLeftReef.Right || fieldTarget == FrontRightReef.Right || fieldTarget == BackReef.Right || fieldTarget == BackLeftReef.Right || fieldTarget == BackRightReef.Right);
    }
    public static boolean isAlgaeReefTarget(FieldTarget fieldTarget){
        return (isLowAlgaeReefTarget(fieldTarget) || isHighAlgaeReefTarget(fieldTarget));
    }
    public static boolean isHighAlgaeReefTarget(FieldTarget fieldTarget){
        return (fieldTarget == FrontLeftReef.Center || fieldTarget == FrontRightReef.Center || fieldTarget == BackReef.Center); 
    }
    public static boolean isLowAlgaeReefTarget(FieldTarget fieldTarget){
        return (fieldTarget == FrontReef.Center || fieldTarget == BackLeftReef.Center || fieldTarget == BackRightReef.Center); 
    }
}
