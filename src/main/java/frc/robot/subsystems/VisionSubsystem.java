package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;


// /*----------------------------------Version 3------------------------------------- */

// public class VisionSubsystem extends SubsystemBase {
//     private final PhotonCamera camera;
//     private PhotonPipelineResult latestResult;

//     private final String cameraName = "GSC_BLACK";

    
//     // Camera position relative to robot center (in meters)
//     private final Translation2d cameraOffset;
//     // x (forward), y (left/right)
//     private final Rotation2d cameraYawOffset; // Camera's yaw relative to robot heading
//     private final double cameraHeight; // z (height) in meters
//     private final double cameraPitch; // Pitch angle in degrees

//     private boolean hasTarget;
//     private double poseAmbiguity;

//     private PhotonTrackedTarget target;
//     private int targetID;
//     private Transform3d bestCameraToTargetPose;
//     private static double rawYaw;
//     private static double target_x;
//     public static double target_y;
//     private double target_z;

//     private static double robotTargetX;
//     private static double robotTargetY;

    
    
//     public VisionSubsystem() {
//         CommandScheduler.getInstance().registerSubsystem(this);
        
//         camera = new PhotonCamera(cameraName);
//         latestResult = new PhotonPipelineResult();
//         cameraOffset = new Translation2d(VisionConstants.cameraXOffset, VisionConstants.cameraYOffset);
//         cameraYawOffset = Rotation2d.fromDegrees(VisionConstants.cameraYawOffset);
//         cameraHeight = VisionConstants.cameraHeight;
//         cameraPitch = VisionConstants.cameraPitch;

//         target_x = 0;
//         target_y = 1;
//     }

//     @Override
//     public void periodic() {
//         // query camera
//         latestResult = camera.getLatestResult();
//         // latestResultG = cameraG.getLatestResult();
//         hasTarget = latestResult.hasTargets();
//         // avoid null pointer exception if no tracked target
//         if ( hasTarget ) {
//             target = latestResult.getBestTarget();
//             targetID = target.getFiducialId();
//             poseAmbiguity = target.getPoseAmbiguity();
//             bestCameraToTargetPose = target.getBestCameraToTarget();

//             // cameraYawOffset is Rotation2d (radians via .getRadians())
//             double yaw = cameraYawOffset.getRadians(); // camera yaw relative to robot
//             double x_cam = bestCameraToTargetPose.getX(); // meters
//             double y_cam = bestCameraToTargetPose.getY();

//             // rotate camera->target by camera yaw, then add cameraOffset to get robot-frame target:
//             double x_robot = cameraOffset.getX() + Math.cos(yaw) * x_cam - Math.sin(yaw) * y_cam;
//             double y_robot = cameraOffset.getY() + Math.sin(yaw) * x_cam + Math.cos(yaw) * y_cam;

//             // store robot-frame values (new getters)
//             robotTargetX = x_robot;
//             robotTargetY = y_robot;


//             rawYaw = target.getYaw();
//             target_x = bestCameraToTargetPose.getX();
//             target_y = bestCameraToTargetPose.getY();
//             target_z = bestCameraToTargetPose.getZ();
//             // mpk - comment out after verifing target values
//             // SmartDashboard.putNumber("targetID",targetID);
//             // SmartDashboard.putNumber("Ambiguity",poseAmbiguity);
//             // SmartDashboard.putNumber("cameraB Yaw", rawYaw);
//             // SmartDashboard.putNumber("target_x", target_x);
//             // SmartDashboard.putNumber("target_y", target_y);
//             // SmartDashboard.putNumber("target_z", target_z);
//         }

//         // rawYaw = latestResultB.getBestTarget().getYaw();

//         // SmartDashboard.putNumber("camera Yaw", rawYawB);
//         SmartDashboard.putNumber("rawYaw", getTarget_rawYaw());
//         SmartDashboard.putBoolean("isLeftAligned", isLeftAlign());
//     }


//     public boolean hasTarget() {
//         return hasTarget;      // mpk - should be periodic result via public gettter/setter interface?
//     }

//     public static double getTarget_rawYaw(){
//         return rawYaw;
//     }

//     public static double getTarget_y() {
//         return target_y;
//     }

//     public static double getTarget_x() {
//         return target_x;
//     }

//     /**
//      * Get yaw adjusted for camera offset
//      * @param robotHeading Current robot heading (Rotation2d)
//      * @return Yaw in degrees relative to robot frame
//      */
//     public double getTargetYawAdjusted(Rotation2d robotHeading) {
//         if (hasTarget()) {
//             double rawYaw = latestResult.getBestTarget().getYaw();
//             // Adjust yaw for camera's orientation and position
//             Rotation2d adjustedYawB = Rotation2d.fromDegrees(rawYaw).plus(cameraYawOffset).minus(robotHeading);
//             return adjustedYawB.getDegrees();
//         }
//         return 0.0;
//     }

//     public boolean isLeftAlign(){
//         if (hasTarget()) {
//             return (rawYaw > VisionConstants.leftAlignRangeLeftInterval) && (rawYaw < VisionConstants.leftAlignRangeRightInterval);
//         }
//         else{
//             return false;
//         }
//     }

//     public double getTargetPitch() {
//         if (hasTarget()) {
//             return latestResult.getBestTarget().getPitch();
//         }
//         return 0.0;
//     }

//     public double getTargetArea() {
//         if (hasTarget()) {
//             return latestResult.getBestTarget().getArea();
//         }
//         return 0.0;
//     }

//     /**
//      * Estimate distance to target, accounting for camera height and pitch
//      * @param targetHeight Height of target from ground (meters)
//      * @return Distance in meters, -1 if no target
//      */
//     public double getDistanceToTarget(double targetHeight) {
//         if (hasTarget()) {
//             double pitch = getTargetPitch();
//             double totalPitch = Math.toRadians(cameraPitch + pitch);
//             return (targetHeight - cameraHeight) / Math.tan(totalPitch);
//         }
//         return -1.0;
//     }

//     public PhotonTrackedTarget getBestTarget() {
//         if (hasTarget()) {
//             return target;
//         }
//         return null;
//     }

//     public Translation2d getCameraOffset() {
//         return cameraOffset;
//     }

//     public Rotation2d getCameraYawOffset() {
//         return cameraYawOffset;
//     }
// }

/* ---------------------------- Version 1.0 (Refactor) ---------------------------- */

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private PhotonPipelineResult latestResult;

    private final String cameraName = "GSC_BLACK";

    // Camera position/orientation relative to robot center
    private final Translation2d cameraOffset;
    private final Rotation2d cameraYawOffset;
    private final double cameraHeight;
    private final double cameraPitch;

    // State
    private boolean hasTarget;
    private double poseAmbiguity;
    private PhotonTrackedTarget target;
    private int targetID;
    private Transform3d bestCameraToTargetPose;

    // Measurements
    private double rawYawDeg;      // raw yaw from Photon (deg)
    private double targetX_robot;  // forward distance (m), robot frame
    private double targetY_robot;  // lateral offset (m), robot frame
    private double targetZ_robot;  // vertical offset (m), robot frame

    public VisionSubsystem() {
        camera = new PhotonCamera(cameraName);
        latestResult = new PhotonPipelineResult();

        cameraOffset = new Translation2d(VisionConstants.cameraXOffset, VisionConstants.cameraYOffset);
        cameraYawOffset = Rotation2d.fromDegrees(VisionConstants.cameraYawOffset);
        cameraHeight = VisionConstants.cameraHeight;
        cameraPitch = VisionConstants.cameraPitch;

        targetX_robot = 0.0;
        targetY_robot = 0.0;
        targetZ_robot = 0.0;
    }

    @Override
    public void periodic() {
        latestResult = camera.getLatestResult();
        hasTarget = latestResult.hasTargets();

        if (hasTarget) {
            target = latestResult.getBestTarget();
            targetID = target.getFiducialId();
            poseAmbiguity = target.getPoseAmbiguity();
            bestCameraToTargetPose = target.getBestCameraToTarget();

            // Camera-frame coordinates (m)
            double x_cam = bestCameraToTargetPose.getX();
            double y_cam = bestCameraToTargetPose.getY();
            double z_cam = bestCameraToTargetPose.getZ();

            rawYawDeg = target.getYaw();

            // Rotate by camera yaw offset to robot frame
            double yawRad = cameraYawOffset.getRadians();
            double x_rot = Math.cos(yawRad) * x_cam - Math.sin(yawRad) * y_cam;
            double y_rot = Math.sin(yawRad) * x_cam + Math.cos(yawRad) * y_cam;

            // Translate by camera offset (robot center → camera)
            targetX_robot = x_rot + cameraOffset.getX();
            targetY_robot = y_rot + cameraOffset.getY();
            targetZ_robot = z_cam;

            // Debug output (optional)
            SmartDashboard.putNumber("targetID", targetID);
            SmartDashboard.putNumber("Ambiguity", poseAmbiguity);
            SmartDashboard.putNumber("rawYaw", rawYawDeg);
            SmartDashboard.putNumber("targetX_robot", targetX_robot);
            SmartDashboard.putNumber("targetY_robot", targetY_robot);
            SmartDashboard.putNumber("targetZ_robot", targetZ_robot);
        }
    }

    // --- Getters ---
    public boolean hasTarget() {
        return hasTarget;
    }

    public double getTargetRawYaw() {
        return rawYawDeg;
    }

    public double getTargetX() {
        return targetX_robot;
    }

    public double getTargetY() {
        return targetY_robot;
    }

    public double getTargetZ() {
        return targetZ_robot;
    }

    public PhotonTrackedTarget getBestTarget() {
        return hasTarget ? target : null;
    }

    public double getTargetPitch() {
        return hasTarget ? latestResult.getBestTarget().getPitch() : 0.0;
    }

    public double getTargetArea() {
        return hasTarget ? latestResult.getBestTarget().getArea() : 0.0;
    }

    /**
     * Estimate distance to target, accounting for camera height and pitch.
     * @param targetHeight height of target from ground (m)
     * @return distance in meters, -1 if no target
     */
    public double getDistanceToTarget(double targetHeight) {
        if (hasTarget) {
            double pitch = getTargetPitch();
            double totalPitch = Math.toRadians(cameraPitch + pitch);
            return (targetHeight - cameraHeight) / Math.tan(totalPitch);
        }
        return -1.0;
    }

    // Camera config accessors
    public Translation2d getCameraOffset() {
        return cameraOffset;
    }

    public Rotation2d getCameraYawOffset() {
        return cameraYawOffset;
    }
}
