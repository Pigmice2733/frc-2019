package frc.robot.motorconfig;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class REV {
    public static void configureNeo(CANSparkMax neo) {
        neo.restoreFactoryDefaults();

        neo.setMotorType(MotorType.kBrushless);
        neo.setIdleMode(IdleMode.kBrake);
        neo.setOpenLoopRampRate(0.12);

        neo.setSmartCurrentLimit(20, 2, 25);
        neo.setSecondaryCurrentLimit(20, 2);

        neo.enableVoltageCompensation(11.0);
    }
}
