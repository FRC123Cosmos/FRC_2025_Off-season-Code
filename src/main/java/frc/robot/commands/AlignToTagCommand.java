package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* -----------------------------------Version 0.75--------------------------------------- */

public class AlignToTagCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;

    private final PIDController yController;    // only strafe
    private final PIDController xController;

    private final double positionTolerance = 0.05; // Meters (5 cm)
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
    private double maxSpeedMultiplier;
    
    public AlignToTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        this(driveSubsystem, visionSubsystem, 0.0, 1);
        this.targetOffsetX = 0.34;
    }

    public AlignToTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, double targetOffsetY) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.targetOffsetY = targetOffsetY;
        this.maxSpeedMultiplier = 1;
        this.targetOffsetX = 0.34;

        this.yController = new PIDController(.90, 0, 1e-4);
        this.xController = new PIDController(.90, 0, 1e-4);
        xController.setTolerance(positionTolerance);
        yController.setTolerance(positionTolerance);

        addRequirements(driveSubsystem, visionSubsystem);
    }


    public AlignToTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, double targetOffsetY, double maxSpeedMultiplier) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.targetOffsetY = targetOffsetY;
        this.maxSpeedMultiplier = maxSpeedMultiplier;
        this.targetOffsetX = 0.34;

        this.yController = new PIDController(.90, 0, 1e-4);
        this.xController = new PIDController(.90, 0, 1e-4);
        xController.setTolerance(positionTolerance);
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

            ySpeed = Math.max(-0.35*maxSpeedMultiplier, Math.min(0.35*maxSpeedMultiplier, pidOutY)); // Meters/sec
            xSpeed = Math.max(-0.35*maxSpeedMultiplier, Math.min(0.35*maxSpeedMultiplier, pidOutX));

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


// /* ---------------------------- Version 1.0 (Refactored) ---------------------------- */

// public class AlignToTagCommand extends Command {
//     private final DriveSubsystem driveSubsystem;
//     private final VisionSubsystem visionSubsystem;

//     private final PIDController yController;
//     private final PIDController xController;
//     private final PIDController thetaController;

//     private final double positionTolerance = 0.05; // meters (5 cm)
//     private final double maxSpeed = 0.3;           // m/s speed cap
//     private final double minEffectiveSpeed = 0.06; // m/s to overcome friction/stiction

//     private final double targetOffsetY;   // lateral offset (m)
//     private final double targetOffsetX;   // forward distance from tag (m)
//     private final double targetYaw = 0.0; // desired facing angle relative to tag (deg)

//     private double targetSetpointY;
//     private double targetSetpointX;
//     private double targetY;
//     private double targetX;
//     private double targetErrorY;
//     private double targetErrorX;
//     private double pidOutY;
//     private double pidOutX;
//     private double ySpeed;
//     private double xSpeed;
//     private double thetaOut;

//     public AlignToTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
//         this(driveSubsystem, visionSubsystem, 0.0, 0.33);
//     }

//     public AlignToTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem,
//                              double targetOffsetY, double targetOffsetX) {
//         this.driveSubsystem = driveSubsystem;
//         this.visionSubsystem = visionSubsystem;
//         this.targetOffsetY = targetOffsetY;
//         this.targetOffsetX = targetOffsetX;

//         this.yController = new PIDController(1.0, 0.0, 0.0);
//         this.xController = new PIDController(1.0, 0.0, 0.0);
//         this.thetaController = new PIDController(0.02, 0.0, 0.001);

//         yController.setTolerance(positionTolerance);
//         xController.setTolerance(positionTolerance);
//         thetaController.setTolerance(.50); // degrees tolerance
//         thetaController.enableContinuousInput(-180.0, 180.0);

//         addRequirements(driveSubsystem, visionSubsystem);
//     }

//     @Override
//     public void initialize() {
//         yController.reset();
//         xController.reset();
//         thetaController.reset();

//         targetErrorY = 0.0;
//         targetErrorX = 0.0;
//         pidOutY = 0.0;
//         pidOutX = 0.0;
//         ySpeed = 0.0;
//         xSpeed = 0.0;
//         thetaOut = 0.0;

//         targetSetpointY = targetOffsetY;
//         targetSetpointX = targetOffsetX;
//     }

//     @Override
//     public void execute() {
//         if (visionSubsystem.hasTarget()) {
//             targetY = visionSubsystem.getTarget_y();
//             targetX = visionSubsystem.getTarget_x();

//             pidOutY = yController.calculate(targetY, targetSetpointY);
//             pidOutX = xController.calculate(targetX, targetSetpointX);
//             targetErrorY = yController.getError();
//             targetErrorX = xController.getError();

//             ySpeed = applySpeedShaping(pidOutY, targetErrorY);
//             xSpeed = applySpeedShaping(pidOutX, targetErrorX);

//             double currentYaw = visionSubsystem.getTarget_rawYaw(); // degrees
//             thetaOut = thetaController.calculate(currentYaw, targetYaw);
//             thetaOut = Math.max(-0.2, Math.min(0.2, thetaOut)); // cap rotation output

//             driveSubsystem.drive(-xSpeed, -ySpeed, thetaOut, false, true);

//         } else {
//             pidOutY = pidOutX = 0.0;
//             ySpeed = xSpeed = 0.0;
//             thetaOut = 0.0;
//             targetErrorY = targetErrorX = 0.0;
//             driveSubsystem.drive(0.0, 0.0, 0.0, false, false);
//         }
//     }

//     @Override
//     public boolean isFinished() {
//         return !visionSubsystem.hasTarget() ||
//                (yController.atSetpoint() && xController.atSetpoint() && thetaController.atSetpoint());
//     }

//     @Override
//     public void end(boolean interrupted) {
//         driveSubsystem.drive(0.0, 0.0, 0.0, false, false);
//     }

//     /**
//      * Shape PID output into usable chassis speed.
//      * - Apply tolerance deadband
//      * - Apply minimum effective speed
//      * - Clip to max speed
//      */
//     private double applySpeedShaping(double pidOutput, double error) {
//         if (Math.abs(error) < positionTolerance) {
//             return 0.0;
//         }
//         double absOut = Math.min(maxSpeed, Math.abs(pidOutput));
//         if (absOut < minEffectiveSpeed) {
//             absOut = minEffectiveSpeed;
//         }
//         return Math.copySign(absOut, pidOutput);
//     }
// }
