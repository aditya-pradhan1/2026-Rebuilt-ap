package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;


public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX motorRight = new TalonFX(RightMotorId);
    private final TalonFX motorLeft = new TalonFX(LeftMotorId);
    private final TalonFX motorKicker = new TalonFX(KickerMotorId);
    private final TalonFX motorReverse = new TalonFX(ReverseMotorId);
    private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    private final CommandSwerveDrivetrain drivetrain;

    public ShooterSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
        m_motorConfig.Slot0 = ShooterGains;
        m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        motorRight.getConfigurator().apply(m_motorConfig);
        motorLeft.getConfigurator().apply(m_motorConfig);
        motorKicker.getConfigurator().apply(m_motorConfig);
        motorReverse.getConfigurator().apply(m_motorConfig);

        motorLeft.setControl(new Follower(RightMotorId, MotorAlignmentValue.Opposed));
        motorKicker.setControl(new Follower(RightMotorId, MotorAlignmentValue.Aligned));
        motorReverse.setControl(new Follower(RightMotorId, MotorAlignmentValue.Opposed));
    }

    // Sets the power of both motors.
    //
    // The left motor is set to the given power, while the right motor is set to the
    // negative of that power to ensure they spin in opposite directions.
    public void setPower(double power) {
        // motorRight.setControl(new DutyCycleOut(power));
        motorRight.setControl(m_request.withVelocity(rps));
    }

    // Disables both motors by setting their power to 0.
    public void stop() {
        motorRight.setControl(m_request.withVelocity(0));
    }

    public void updateShotVisualization(double v0_mag, double launchAngleDegrees) {
        //this whole method is AI generated so not really sure if its gonna work lol
        //update: it works, i'm gonna fine tune it now
        
        // 1. Fetch robot pose (The starting point)
        Pose2d robotPose = drivetrain.getState().Pose;
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();

        // placeholders cuz i can't fetch them right now
        double robotVx = 0;
        double robotVy = 0;

        // 2. Calculate the Shot's initial velocity (without robot movement)
        double launchAngleRad = Math.toRadians(launchAngleDegrees);
        double headingRad = robotPose.getRotation().getRadians();

        double shotVx = v0_mag * Math.cos(launchAngleRad) * Math.cos(headingRad);
        double shotVy = v0_mag * Math.cos(launchAngleRad) * Math.sin(headingRad);
        double shotVz = v0_mag * Math.sin(launchAngleRad);

        // 3. Combine them!
        double totalVx = shotVx + robotVx;
        double totalVy = shotVy + robotVy;
        double totalVz = shotVz;

        // 3. Create a list of Poses (the "streak")
        int samples = 20; // How many points to draw
        double dt = 0.1;  // Time step between points (0.1 seconds)
        Pose3d[] trajectory = new Pose3d[samples];

        for (int i = 0; i < samples; i++) {
            double t = i * dt;
            
            // 4. Basic Kinematics (x = x0 + vt, z = z0 + vt - 0.5gt^2)
            double currX = robotX + (totalVx * t);
            double currY = robotY + (totalVy * t);
            double currZ = 0.5 + (totalVz * t) - (0.5 * 9.81 * t * t); // Starting 0.5m high

            // Clamp Z so it doesn't go through the floor in the visualizer
            if (currZ < 0) currZ = 0;

            // Create the Pose3d Object
            trajectory[i] = new Pose3d(currX, currY, currZ, new Rotation3d());
        }

        // 5. Upload to Network Tables (Using the modern Struct approach)
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("Shooter/BallTrajectory", Pose3d.struct)
            .publish()
            .set(trajectory);
    }

    public void clearTrajectory() {
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("Shooter/BallTrajectory", Pose3d.struct)
            .publish()
            .set(new Pose3d[0]); 
    }
}
