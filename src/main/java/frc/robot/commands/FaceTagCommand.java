package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;


public class FaceTagCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;

    private final PIDController thetaController;

    private final double angleTolerance = 1;    // Degree 
    private double targetOffsetTheta; 
    private double targetSetpointTheta; 
    private double targetTheta;
    private double targetErrorTheta;
    private double pidOutTheta;
    private double thetaSpeed;
    
    public FaceTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        this(driveSubsystem, visionSubsystem, 0.0);
        
        addRequirements(driveSubsystem, visionSubsystem);
    }

    public FaceTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, double targetOffsetTheta) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.targetOffsetTheta = targetOffsetTheta;

        this.thetaController = new PIDController(1, 0.0, 0.0);
        thetaController.enableContinuousInput(-180,180);
        thetaController.setTolerance(angleTolerance);

        addRequirements(driveSubsystem, visionSubsystem);
    }

    @Override
    public void initialize() {
        thetaController.reset();
        targetErrorTheta = 0.0;
        pidOutTheta = 0.0;
        targetSetpointTheta = 0;
        thetaSpeed = 0.0;
    }

    @Override
    public void execute() {
        if (visionSubsystem.hasTarget()) {

            targetTheta = VisionSubsystem.getTarget_rawYaw();
            pidOutTheta = thetaController.calculate(targetTheta, targetSetpointTheta);
            targetErrorTheta = thetaController.getError();

            thetaSpeed = Math.max(-.1, Math.min(.1, pidOutTheta)); // degrees/sec


            driveSubsystem.drive(0, 0, thetaSpeed, false, true);    // try fieldrelative true?

        } else {
            pidOutTheta = 0.0;
            thetaSpeed = 0.0;
            targetErrorTheta = 0.0;
            driveSubsystem.drive(0.0, 0.0, 0.0, false, false);
        }
    }

    @Override
    public boolean isFinished() {
        if (!visionSubsystem.hasTarget() || (thetaController.atSetpoint())) {
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

