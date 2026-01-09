package frc.robot.SubSystem.Swerve.Gyro;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

public class NavXGyro implements GyroIO{

    AHRS navX;
    public NavXGyro() {
        navX = new AHRS(NavXComType.kMXP_SPI, 100);
    }

    @Override
    public double getAngleRad() {
        return navX.getRotation2d().getRadians();
    }
}
