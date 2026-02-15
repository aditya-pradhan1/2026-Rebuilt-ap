package frc.robot.subsystems;

import static frc.robot.Constants.SlapdownConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SlapdownSubsystem extends SubsystemBase{
    private final TalonFX motorSlapdown = new TalonFX(SlapdownMotorId);
    // private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
    public boolean isSlapdownDeployed = false;

    public SlapdownSubsystem() {
        TalonFXConfiguration m_motorConfig = new TalonFXConfiguration();
        m_motorConfig.Slot0 = IntakeGains;
        m_motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        motorSlapdown.getConfigurator().apply(m_motorConfig);
    }

    //Sets the power of the slapdown motor.
    public void slapdown(double power) {
        //rps prolly needs to be multiplied with some constant
        motorSlapdown.setControl(m_request.withPosition(.25));
        isSlapdownDeployed = true;
    }

    //Retracts the slapdown.
    public void retractSlapdown(double power) {
        //rps prolly needs to be multiplied with some constant
        motorSlapdown.setControl(m_request.withPosition(-.25));
        isSlapdownDeployed = false;
    }

    public boolean getSlapdownDeployed() {
        return isSlapdownDeployed;
    }

    // public void stop() {
    //     motorSlapdown.setControl(m_velocityRequest.withVelocity(0));
    // }
}
