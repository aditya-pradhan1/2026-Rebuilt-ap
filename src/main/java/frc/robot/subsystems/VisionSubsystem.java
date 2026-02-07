package frc.robot.subsystems;

import java.io.IOException;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private final CommandSwerveDrivetrain drivetrain;
    // not final because setting fieldLayout wasn't working without try/catch
    private AprilTagFieldLayout fieldLayout;

    private VisionSystemSim visionSim;
    private PhotonCameraSim cameraSim;
    //private final StructSubscriber<Pose2d> poseSub;

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {

        this.drivetrain = drivetrain;
        // put the real camera when connecting to robot, but for simulation I'm just making a random name
        camera = new PhotonCamera("sim-camera");
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2026RebuiltAndymark.m_resourceFile);
        } catch (IOException e) {
            System.out.println("Could not find the field file!");
        }
        
        // put real values in here when cad + camera location is finalized
        // these are rough estimates based on current design team predictions (jan 26)
        Transform3d robotToCam = new Transform3d(new Translation3d(0.2, 0.0, 0.45), new Rotation3d(0, 0, 0));

        if (fieldLayout != null) {
            poseEstimator = new PhotonPoseEstimator(fieldLayout, robotToCam);
        } else {
            poseEstimator = null;
        }

        if (RobotBase.isSimulation()) {
            visionSim = new VisionSystemSim("main");
            if (fieldLayout != null) {
                visionSim.addAprilTags(fieldLayout);
            }
            SimCameraProperties cameraProp = new SimCameraProperties(); 

            cameraProp.setCalibration(640,480,Rotation2d.fromDegrees(100));
            cameraProp.setCalibError(.25, 0.88);
            cameraProp.setFPS(60);
            cameraProp.setAvgLatencyMs(35);
            cameraProp.setLatencyStdDevMs(5);
            cameraSim = new PhotonCameraSim(camera, cameraProp);
            cameraSim.enableRawStream(true);
            cameraSim.enableProcessedStream(true);

            visionSim.addCamera(cameraSim, robotToCam);
            /*poseSub = NetworkTableInstance.getDefault()
                .getStructTopic("SmartDashboard/RobotPose", Pose2d.struct)
                .subscribe(new Pose2d());
            */
            SmartDashboard.putData("VisionSim", visionSim.getDebugField());
        } else {
            visionSim = null;
            cameraSim = null;
        }

    }
    
    @Override
    public void simulationPeriodic() {
        visionSim.update(drivetrain.getPose());
    }

}