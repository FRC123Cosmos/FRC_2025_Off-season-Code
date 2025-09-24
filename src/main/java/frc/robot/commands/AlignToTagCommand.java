package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* --------------------------------------------------Version 0------------------------------------------------------ */
// public class AlignToTagCommand extends Command {
//     private final DriveSubsystem driveSubsystem;
//     private final VisionSubsystem visionSubsystem;
//     private final Pose2d targetOffset;
    
//     private final double strafeSpeedFactor = 0.3;    // Proportional gain for strafing
//     private final double positionTolerance = 0.02;   // Meters (5 cm)
//     private final double maxStrafeSpeed = 0.5;       // Meters/sec max speed

//     public AlignToTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
//         this(driveSubsystem, visionSubsystem, new Pose2d(0.47, 0.0, Rotation2d.fromDegrees(0.0)));
//     }

//     public AlignToTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, Pose2d targetOffset) {
//         this.driveSubsystem = driveSubsystem;
//         this.visionSubsystem = visionSubsystem;
//         this.targetOffset = targetOffset;
        
//         addRequirements(driveSubsystem, visionSubsystem);
//     }

//     @Override
//     public void initialize() {
//         LedSubsystem.setPurpleMsg();
//     }

//     @Override
//     public void execute() {
//         if (visionSubsystem.hasTarget()) {
//             PhotonTrackedTarget target = visionSubsystem.getBestTarget();
//             Transform3d cameraToTarget = target.getBestCameraToTarget();
            
//             // Get target pose relative to camera
//             Pose2d targetPoseRelative = new Pose2d(
//                 cameraToTarget.getX(),
//                 cameraToTarget.getY(),
//                 Rotation2d.fromDegrees(visionSubsystem.getTargetYawAdjusted(driveSubsystem.getHeadingRotation2d()))
//             );

//             // Transform to robot-relative pose
//             Translation2d cameraOffset = visionSubsystem.getCameraOffset();
//             Pose2d targetPoseRobotRelative = targetPoseRelative.transformBy(
//                 new Transform2d(cameraOffset, visionSubsystem.getCameraYawOffset())
//             );

//             // Calculate desired pose with offset
//             Pose2d desiredPose = targetPoseRobotRelative.transformBy(
//                 new Transform2d(targetOffset.getTranslation().unaryMinus(), targetOffset.getRotation().unaryMinus())
//             );

//             Pose2d currentPose = driveSubsystem.getEstPose2d();

//             // Calculate strafe error (only Y-axis)
//             double yError = desiredPose.getY() - currentPose.getY();

//             // Simple proportional control for strafing
//             double ySpeed = yError * strafeSpeedFactor;
//             ySpeed = Math.max(-maxStrafeSpeed, Math.min(maxStrafeSpeed, ySpeed));

//             // Drive only in Y direction (strafing), no X movement or rotation
//             driveSubsystem.drive(0.0, ySpeed, 0.0, false, false);

//             // Update vision measurement
//             Pose2d visionPose = targetPoseRobotRelative;
//             double timestamp = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
//             driveSubsystem.addVisionMeasurement(visionPose, timestamp);
//         } else {
//             driveSubsystem.drive(0.0, 0.0, 0.0, false, false);
//         }
//     }

//     @Override
//     public boolean isFinished() {
//         if (!visionSubsystem.hasTarget()) {
//             LedSubsystem.setPurpleMsg();
//             return true;
//         }

//         // Same pose calculations as execute()
//         PhotonTrackedTarget target = visionSubsystem.getBestTarget();
//         Transform3d cameraToTarget = target.getBestCameraToTarget();
//         Pose2d targetPoseRelative = new Pose2d(
//             cameraToTarget.getX(),
//             cameraToTarget.getY(),
//             Rotation2d.fromDegrees(visionSubsystem.getTargetYawAdjusted(driveSubsystem.getHeadingRotation2d()))
//         );
//         Translation2d cameraOffset = visionSubsystem.getCameraOffset();
//         Pose2d targetPoseRobotRelative = targetPoseRelative.transformBy(
//             new Transform2d(cameraOffset, visionSubsystem.getCameraYawOffset())
//         );
//         Pose2d desiredPose = targetPoseRobotRelative.transformBy(
//             new Transform2d(targetOffset.getTranslation().unaryMinus(), targetOffset.getRotation().unaryMinus())
//         );

//         Pose2d currentPose = driveSubsystem.getEstPose2d();
//         double yError = Math.abs(desiredPose.getY() - currentPose.getY());

//         return yError < positionTolerance;
//     }

//     @Override
//     public void end(boolean interrupted) {
//         driveSubsystem.drive(0.0, 0.0, 0.0, false, false);
//         LedSubsystem.setAllianceSolid();
//     }
// }

/* --------------------------------------------------Version 1---------------------------------------------------- */

// public class AlignToTagCommand extends Command {
//     private final DriveSubsystem driveSubsystem;
//     private final VisionSubsystem visionSubsystem;
//     private final Pose2d targetOffset; 

//     private final PIDController xController;
//     private final PIDController yController;
//     private final PIDController thetaController;

//     private final double positionTolerance = 0.03; // Meters (5 cm)
//     private final double rotationTolerance = 1.0; // Degrees

//     public AlignToTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
//         this(driveSubsystem, visionSubsystem, new Pose2d(0.35, 0.0, Rotation2d.fromDegrees(0.0)));
//     }

//     public AlignToTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, Pose2d targetOffset) {
//         this.driveSubsystem = driveSubsystem;
//         this.visionSubsystem = visionSubsystem;
//         this.targetOffset = targetOffset;

//         this.xController = new PIDController(0.50, 0.0, 0.0); 
//         this.yController = new PIDController(1.0, 0.0, 0.1);
//         this.thetaController = new PIDController(0.0, 0.0, 0.0);

//         xController.setTolerance(positionTolerance);
//         yController.setTolerance(positionTolerance);
//         thetaController.setTolerance(rotationTolerance);

//         thetaController.enableContinuousInput(-180.0, 180.0);

//         addRequirements(driveSubsystem, visionSubsystem);
//     }

//     @Override
//     public void initialize() {
//         xController.reset();
//         yController.reset();
//         thetaController.reset();
//     }

//     @Override
//     public void execute() {
//         if (visionSubsystem.hasTarget()) {
//             PhotonTrackedTarget target = visionSubsystem.getBestTarget();
//             Transform3d cameraToTarget = target.getBestCameraToTarget();
//             Pose2d targetPoseRelative = new Pose2d(
//                 cameraToTarget.getX(),
//                 cameraToTarget.getY(),
//                 Rotation2d.fromDegrees(visionSubsystem.getTargetYawAdjusted(driveSubsystem.getHeadingRotation2d()))
//             );

//             Translation2d cameraOffset = visionSubsystem.getCameraOffset();
//             Pose2d targetPoseRobotRelative = targetPoseRelative.transformBy(
//                 new Transform2d(cameraOffset, visionSubsystem.getCameraYawOffset())
//             );

//             Pose2d desiredPose = targetPoseRobotRelative.transformBy(
//                 new Transform2d(targetOffset.getTranslation().unaryMinus(), targetOffset.getRotation().unaryMinus())
//             );

//             Pose2d currentPose = driveSubsystem.getEstPose2d();

//             double xSpeed = xController.calculate(currentPose.getX(), desiredPose.getX());
//             double ySpeed = yController.calculate(currentPose.getY(), desiredPose.getY());
//             double rotationSpeed = thetaController.calculate(
//                 currentPose.getRotation().getDegrees(),
//                 desiredPose.getRotation().getDegrees()
//             );

//             xSpeed = Math.max(-0.5, Math.min(0.5, xSpeed)); // Meters/sec
//             ySpeed = Math.max(-0.5, Math.min(0.5, ySpeed)); // Meters/sec
//             rotationSpeed = Math.max(-0.5, Math.min(0.5, rotationSpeed)); // Radians/sec

//             driveSubsystem.drive(xSpeed, ySpeed, rotationSpeed, false, false);

//             Pose2d visionPose = targetPoseRobotRelative;
//             double timestamp = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
//             driveSubsystem.addVisionMeasurement(visionPose, timestamp);
//         } else {
//             driveSubsystem.drive(0.0, 0.0, 0.0, false, false);
//         }
//     }

//     @Override
//     public boolean isFinished() {
//         if (!visionSubsystem.hasTarget()) {
//             return true; 
//         }

//         Pose2d currentPose = driveSubsystem.getEstPose2d();
//         PhotonTrackedTarget target = visionSubsystem.getBestTarget();
//         Transform3d cameraToTarget = target.getBestCameraToTarget();
//         Pose2d targetPoseRelative = new Pose2d(
//             cameraToTarget.getX(),
//             cameraToTarget.getY(),
//             Rotation2d.fromDegrees(visionSubsystem.getTargetYawAdjusted(driveSubsystem.getHeadingRotation2d()))
//         );
//         Translation2d cameraOffset = visionSubsystem.getCameraOffset();
//         Pose2d targetPoseRobotRelative = targetPoseRelative.transformBy(
//             new Transform2d(cameraOffset, visionSubsystem.getCameraYawOffset())
//         );
//         Pose2d desiredPose = targetPoseRobotRelative.transformBy(
//             new Transform2d(targetOffset.getTranslation().unaryMinus(), targetOffset.getRotation().unaryMinus())
//         );

//         return xController.atSetpoint() && 
//                yController.atSetpoint() && 
//                thetaController.atSetpoint();
//     }

//     @Override
//     public void end(boolean interrupted) {
//         driveSubsystem.drive(0.0, 0.0, 0.0, false, false);
//     }
// }

// /* --------------------------------------------------Version 0.5 :-) ------------------------------------------------------ */

// public class AlignToTagCommand extends Command {
//     private final DriveSubsystem driveSubsystem;
//     private final VisionSubsystem visionSubsystem;

//     private final PIDController yController;    // only strafe

//     private final double positionTolerance = 0.05; // Meters (5 cm)
//     private double targetOffset;        // desired static position from center of target
//     private double targetSetpoint;      // setpoint fed to PID
//     private double targetY;             // actual position from photonvision
//     private double targetError;        // error input to PID
//     private double pidOut;             // PID control output
//     private double ySpeed;          // clipped PID control output
    
//         public AlignToTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
//             this(driveSubsystem, visionSubsystem, 0.0 );
//         }
    
//         public AlignToTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, double targetOffset) {
//             this.driveSubsystem = driveSubsystem;
//             this.visionSubsystem = visionSubsystem;
//             this.targetOffset = targetOffset;

//             this.yController = new PIDController(1.0, 0.0, 0.0);
//             yController.setTolerance(positionTolerance);

//             addRequirements(driveSubsystem, visionSubsystem);
//     }

//     @Override
//     public void initialize() {
//         yController.reset();
//         targetError = 0.0;
//         pidOut = 0.0;
//         targetSetpoint = 1;
//         ySpeed = 0.0;
//         // SmartDashboard.putNumber("targetSetpoint", targetSetpoint);
//         // SmartDashboard.putNumber("target_y", targetY);
//         // SmartDashboard.putNumber("targetError", targetError);
//         // SmartDashboard.putNumber("pidOut", pidOut);
//         // SmartDashboard.putNumber("ySpeed", ySpeed);
//     }

//     @Override
//     public void execute() {
//         if (visionSubsystem.hasTarget()) {

//             targetSetpoint = targetOffset;      // static command
//             targetY = VisionSubsystem.getTarget_y();
//             pidOut = yController.calculate(targetY,targetSetpoint);
//             targetError = yController.getError();

//             ySpeed = Math.max(-0.25, Math.min(0.25, pidOut)); // Meters/sec

//             driveSubsystem.drive(0, -ySpeed, 0, false, true);    // try kinematic rate limit??

//         } else {
//             pidOut = 0.0;
//             ySpeed = 0.0;
//             targetError = 0.0;
//             driveSubsystem.drive(0.0, 0.0, 0.0, false, false);
//         }
//         // mpk - comment out after verifing target values
//         // SmartDashboard.putNumber("targetSetpoint", targetSetpoint);
//         // SmartDashboard.putNumber("target_y", targetY);
//         // SmartDashboard.putNumber("targetError", targetError);
//         // SmartDashboard.putNumber("pidOut", pidOut);
//         // SmartDashboard.putNumber("ySpeed", ySpeed);

//     }

//     @Override
//     public boolean isFinished() {
//         if (!visionSubsystem.hasTarget() || yController.atSetpoint()) {
//             return true; 
//         } else {
//             return false;
//         }

//     }

//     @Override
//     public void end(boolean interrupted) {
//         driveSubsystem.drive(0.0, 0.0, 0.0, false, false);
//     }
// }


/* -----------------------------------Version 0.75--------------------------------------- */

public class AlignToTagCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;

    private final PIDController yController;    // only strafe
    private final PIDController xController;

    private final double positionTolerance = 0.01; // Meters (5 cm)
    private double targetOffsetY;        // desired static position from center of target
    private double targetOffsetX; 
    private double targetSetpointY;      // setpoint fed to PID
    private double targetSetpointX; 
    private double targetY;             // actual position from photonvision
    private double targetX;
    private double targetErrorY;        // error input to PID
    private double targetErrorX;
    private double pidOutY;             // PID control output
    private double pidOutX; 
    private double ySpeed;          // clipped PID control output
    private double xSpeed; 
    
        public AlignToTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
            this(driveSubsystem, visionSubsystem, 0.0);
            targetOffsetX = 0.33;
        }
    
        public AlignToTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, double targetOffsetY) {
            this.driveSubsystem = driveSubsystem;
            this.visionSubsystem = visionSubsystem;
            this.targetOffsetY = targetOffsetY;
            targetOffsetX = 0.33;

            this.yController = new PIDController(1.0, 0, 0.0);
            this.xController = new PIDController(1.0, 0, 0.0);
            yController.setTolerance(positionTolerance);

            addRequirements(driveSubsystem, visionSubsystem);
    }

    @Override
    public void initialize() {
        yController.reset();
        xController.reset();
        targetErrorY = 0.0;
        targetErrorX = 0.0;
        pidOutY = 0.0;
        pidOutX = 0.0;
        targetSetpointY = 1;
        targetSetpointX = 0.0;
        ySpeed = 0.0;
        xSpeed = 0.0;
        // SmartDashboard.putNumber("targetSetpoint", targetSetpoint);
        // SmartDashboard.putNumber("target_y", targetY);
        // SmartDashboard.putNumber("targetError", targetError);
        // SmartDashboard.putNumber("pidOut", pidOut);
        // SmartDashboard.putNumber("ySpeed", ySpeed);
    }

    @Override
    public void execute() {
        if (visionSubsystem.hasTarget()) {

            targetSetpointY = targetOffsetY;      // static command
            targetSetpointX = targetOffsetX;
            targetY = VisionSubsystem.getTarget_y();
            targetX = VisionSubsystem.getTarget_x();
            pidOutY = yController.calculate(targetY, targetSetpointY);
            pidOutX = xController.calculate(targetX, targetSetpointX);
            targetErrorY = yController.getError();
            targetErrorX = xController.getError();

            ySpeed = Math.max(-0.2, Math.min(0.2, pidOutY)); // Meters/sec
            xSpeed = Math.max(-0.2, Math.min(0.2, pidOutX));

            driveSubsystem.drive(-xSpeed, -ySpeed, 0, false, true);    // try kinematic rate limit??

        } else {
            pidOutY = 0.0;
            pidOutX = 0.0;
            ySpeed = 0.0;
            xSpeed = 0.0;
            targetErrorY = 0.0;
            targetErrorX = 0.0;
            driveSubsystem.drive(0.0, 0.0, 0.0, false, false);
        }
        // mpk - comment out after verifing target values
        // SmartDashboard.putNumber("targetSetpoint", targetSetpoint);
        // SmartDashboard.putNumber("target_y", targetY);
        // SmartDashboard.putNumber("targetError", targetError);
        // SmartDashboard.putNumber("pidOut", pidOut);
        // SmartDashboard.putNumber("ySpeed", ySpeed);

    }

    @Override
    public boolean isFinished() {
        if (!visionSubsystem.hasTarget() || (yController.atSetpoint() && xController.atSetpoint())) {
            return true; 
        } else {
            return false;
        }

    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0.0, 0.0, 0.0, false, false);
    }
}



// /* -----------------------------------Version 0.85--------------------------------------- */

// public class AlignToTagCommand extends Command {
//     private final DriveSubsystem driveSubsystem;
//     private final VisionSubsystem visionSubsystem;

//     private final PIDController yController;    // only strafe
//     private final PIDController xController;
//     private final PIDController thetaController;

//     private final double positionTolerance = 0.04; // Meters (4 cm)
//     private final double angleTolerance = 2;    // Degree 
//     private double targetOffsetY;        // desired static position from center of target
//     private double targetOffsetX; 
//     private double targetOffsetTheta; 
//     private double targetSetpointY;      // setpoint fed to PID
//     private double targetSetpointX; 
//     private double targetSetpointTheta; 
//     private double targetY;             // actual position from photonvision
//     private double targetX;
//     private double targetTheta;
//     private double targetErrorY;        // error input to PID
//     private double targetErrorX;
//     private double targetErrorTheta;
//     private double pidOutY;             // PID control output
//     private double pidOutX; 
//     private double pidOutTheta;
//     private double ySpeed;          // clipped PID control output
//     private double xSpeed; 
//     private double thetaSpeed;
    
//     public AlignToTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
//         this(driveSubsystem, visionSubsystem, 0.0);
//         targetOffsetX = 0.33;
//         targetOffsetTheta = 0;
//     }

//     public AlignToTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, double targetOffsetY) {
//         this.driveSubsystem = driveSubsystem;
//         this.visionSubsystem = visionSubsystem;
//         this.targetOffsetY = targetOffsetY;
//         targetOffsetX = 0.33;
//         targetOffsetTheta = 0;

//         this.yController = new PIDController(1.0, 0.0, 0.0);
//         this.xController = new PIDController(1.0, 0.0, 0.0);
//         this.thetaController = new PIDController(1.0, 0.0, 0.0);
//         yController.setTolerance(positionTolerance);
//         xController.setTolerance(positionTolerance);
//         thetaController.setTolerance(angleTolerance);

//         addRequirements(driveSubsystem, visionSubsystem);
//     }

//     @Override
//     public void initialize() {
//         yController.reset();
//         xController.reset();
//         targetErrorY = 0.0;
//         targetErrorX = 0.0;
//         targetErrorTheta = 0.0;
//         pidOutY = 0.0;
//         pidOutX = 0.0;
//         pidOutTheta = 0.0;
//         targetSetpointY = 1;
//         targetSetpointX = 0.0;
//         targetSetpointTheta = 25;
//         ySpeed = 0.0;
//         xSpeed = 0.0;
//         thetaSpeed = 0.0;
//         // SmartDashboard.putNumber("targetSetpoint", targetSetpoint);
//         // SmartDashboard.putNumber("target_y", targetY);
//         // SmartDashboard.putNumber("targetError", targetError);
//         // SmartDashboard.putNumber("pidOut", pidOut);
//         // SmartDashboard.putNumber("ySpeed", ySpeed);
//     }

//     @Override
//     public void execute() {
//         if (visionSubsystem.hasTarget()) {

//             targetSetpointY = targetOffsetY;      // static command
//             targetSetpointX = targetOffsetX;
//             targetSetpointTheta = targetSetpointTheta;
//             targetY = VisionSubsystem.getTarget_y();
//             targetX = VisionSubsystem.getTarget_x();
//             targetTheta = VisionSubsystem.getTarget_rawYaw();
//             pidOutY = yController.calculate(targetY, targetSetpointY);
//             pidOutX = xController.calculate(targetX, targetSetpointX);
//             pidOutTheta = thetaController.calculate(targetTheta, targetSetpointTheta);
//             targetErrorY = yController.getError();
//             targetErrorX = xController.getError();
//             targetErrorTheta = thetaController.getError();

//             ySpeed = Math.max(-0.25, Math.min(0.25, pidOutY)); // Meters/sec
//             xSpeed = Math.max(-0.25, Math.min(0.25, pidOutX));
//             thetaSpeed = Math.max(-.1, Math.min(.1, pidOutTheta)); // degrees/sec


//             driveSubsystem.drive(-xSpeed, -ySpeed, -thetaSpeed, false, true);    // try kinematic rate limit??

//         } else {
//             pidOutY = 0.0;
//             pidOutX = 0.0;
//             pidOutTheta = 0.0;
//             ySpeed = 0.0;
//             xSpeed = 0.0;
//             thetaSpeed = 0.0;
//             targetErrorY = 0.0;
//             targetErrorX = 0.0;
//             targetErrorTheta = 0.0;
//             driveSubsystem.drive(0.0, 0.0, 0.0, false, false);
//         }
//         // mpk - comment out after verifing target values
//         // SmartDashboard.putNumber("targetSetpoint", targetSetpoint);
//         // SmartDashboard.putNumber("target_y", targetY);
//         // SmartDashboard.putNumber("targetError", targetError);
//         // SmartDashboard.putNumber("pidOut", pidOut);
//         // SmartDashboard.putNumber("ySpeed", ySpeed);

//     }

//     @Override
//     public boolean isFinished() {
//         if (!visionSubsystem.hasTarget() || (yController.atSetpoint() && xController.atSetpoint() && (thetaController.atSetpoint()))) {
//             return true; 
//         } else {
//             return false;
//         }

//     }

//     @Override
//     public void end(boolean interrupted) {
//         driveSubsystem.drive(0.0, 0.0, 0.0, false, false);
//     }
// }



/*------------------------------------Version 2----------------------------------------*/

// public class AlignToTagCommand extends Command {
//     private final DriveSubsystem driveSubsystem;
//     private final VisionSubsystem visionSubsystem;
//     private final Pose2d targetOffset; 
//     private final double positionTolerance = 0.01;
//     private final double rotationTolerance = 2.0; 
//     private final double translationSpeedFactor = 0.5; 
//     private final double rotationSpeedFactor = 0.02; 

//     public AlignToTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
//         this.driveSubsystem = driveSubsystem;
//         this.visionSubsystem = visionSubsystem;
//         this.targetOffset = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)); 
//         addRequirements(driveSubsystem, visionSubsystem);
//     }

//     public AlignToTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, Pose2d targetOffset) {
//         this.driveSubsystem = driveSubsystem;
//         this.visionSubsystem = visionSubsystem;
//         this.targetOffset = targetOffset; 
//         addRequirements(driveSubsystem, visionSubsystem);
//     }

//     @Override
//     public void initialize() {
//     }

//     @Override
//     public void execute() {
//         if (visionSubsystem.hasTarget()) {
//             PhotonTrackedTarget target = visionSubsystem.getBestTarget();
//             Transform3d cameraToTarget = target.getBestCameraToTarget();
            
//             Pose2d targetPoseRelative = new Pose2d(
//                 cameraToTarget.getX(),
//                 cameraToTarget.getY(),
//                 Rotation2d.fromDegrees(visionSubsystem.getTargetYawAdjusted(driveSubsystem.getHeadingRotation2d()))
//             );

//             Translation2d cameraOffset = visionSubsystem.getCameraOffset();
//             Pose2d targetPoseRobotRelative = targetPoseRelative.transformBy(
//                 new Transform2d(cameraOffset, visionSubsystem.getCameraYawOffset())
//             );

//             Pose2d desiredPose = targetPoseRobotRelative.transformBy(
//                 new Transform2d(targetOffset.getTranslation().unaryMinus(), targetOffset.getRotation().unaryMinus())
//             );

//             Pose2d currentPose = driveSubsystem.getEstPose2d();

//             double xError = desiredPose.getX() - currentPose.getX();
//             double yError = desiredPose.getY() - currentPose.getY();
//             double thetaError = desiredPose.getRotation().minus(currentPose.getRotation()).getDegrees();

//             double xSpeed = xError * translationSpeedFactor;
//             double ySpeed = yError * translationSpeedFactor;
//             double rotationSpeed = thetaError * rotationSpeedFactor;

//             xSpeed = Math.max(-0.5, Math.min(0.5, xSpeed)); // Meters/sec
//             ySpeed = Math.max(-0.5, Math.min(0.5, ySpeed)); // Meters/sec
//             rotationSpeed = Math.max(-0.5, Math.min(0.5, rotationSpeed)); // Radians/sec

//             driveSubsystem.drive(xSpeed, ySpeed, rotationSpeed, false, false);

//             Pose2d visionPose = targetPoseRobotRelative; // Could refine this further
//             double timestamp = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
//             driveSubsystem.addVisionMeasurement(visionPose, timestamp);
//         } else {
//             driveSubsystem.drive(0.0, 0.0, 0.0, false, false);
//         }
//     }

//     @Override
//     public boolean isFinished() {
//         if (!visionSubsystem.hasTarget()) {
//             return true;
//         }

//         Pose2d currentPose = driveSubsystem.getEstPose2d();
//         PhotonTrackedTarget target = visionSubsystem.getBestTarget();
//         Transform3d cameraToTarget = target.getBestCameraToTarget();
//         Pose2d targetPoseRelative = new Pose2d(
//             cameraToTarget.getX(),
//             cameraToTarget.getY(),
//             Rotation2d.fromDegrees(visionSubsystem.getTargetYawAdjusted(driveSubsystem.getHeadingRotation2d()))
//         );
//         Translation2d cameraOffset = visionSubsystem.getCameraOffset();
//         Pose2d targetPoseRobotRelative = targetPoseRelative.transformBy(
//             new Transform2d(cameraOffset, visionSubsystem.getCameraYawOffset())
//         );
//         Pose2d desiredPose = targetPoseRobotRelative.transformBy(
//             new Transform2d(targetOffset.getTranslation().unaryMinus(), targetOffset.getRotation().unaryMinus())
//         );

//         double xError = Math.abs(desiredPose.getX() - currentPose.getX());
//         double yError = Math.abs(desiredPose.getY() - currentPose.getY());
//         double thetaError = Math.abs(desiredPose.getRotation().minus(currentPose.getRotation()).getDegrees());

//         return xError < positionTolerance && yError < positionTolerance && thetaError < rotationTolerance;
//     }

//     @Override
//     public void end(boolean interrupted) {
//         driveSubsystem.drive(0.0, 0.0, 0.0, false, false);
//     }
// }