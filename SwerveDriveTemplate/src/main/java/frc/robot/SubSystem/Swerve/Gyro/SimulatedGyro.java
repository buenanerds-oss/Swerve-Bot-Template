package frc.robot.SubSystem.Swerve.Gyro;

import org.ironmaple.simulation.drivesims.GyroSimulation;

public class SimulatedGyro implements GyroIO{

    GyroSimulation gyro;
    public SimulatedGyro(GyroSimulation gyro) {
        this.gyro = gyro;
    }

    @Override
    public double getAngleRad() {
        return gyro.getGyroReading().getRadians();
    }
}
