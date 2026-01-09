package frc.robot.SubSystem.Swerve;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedMotorController.GenericMotorController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.SubSystem.Logging.GroupLogger;
import frc.robot.Util.BasicUtil;

public class ModuleSim implements ModuleIO{

    double turnVolts;
    double driveVolts;
    

    GenericMotorController DriveMotor;
    GenericMotorController turnMotor;
    PIDController drivePID = new PIDController(ModuleMotorConfig.SIM_DRIVE_P, ModuleMotorConfig.SIM_DRIVE_I, ModuleMotorConfig.SIM_DRIVE_D);
    PIDController turnPID = new PIDController(ModuleMotorConfig.SIM_TURN_P, ModuleMotorConfig.SIM_TURN_I, ModuleMotorConfig.SIM_TURN_D);

    SwerveModuleSimulationConfig moduleConfig = COTS.ofMark4(DCMotor.getNeoVortex(1), DCMotor.getNeoVortex(1), 1.2, 2); // gear ratio level is the L2 of the Mark4 documentation
    DriveTrainSimulationConfig drivetrainConfig = DriveTrainSimulationConfig.Default().withCustomModuleTranslations(SwerveConstants.moduletranslations)
    .withRobotMass(Kilograms.of(SwerveConstants.mass_KG))
    .withGyro(COTS.ofNav2X())
    .withSwerveModule(moduleConfig);
    SwerveDriveSimulation swerveSim = new SwerveDriveSimulation(drivetrainConfig, SwerveConstants.initialPose);

    int index;
    SwerveModuleState currentSwerveState;
    public ModuleSim(int index) {
        this.index = index;

        DriveMotor = swerveSim.getModules()[index].useGenericMotorControllerForDrive();
        turnMotor = swerveSim.getModules()[index].useGenericControllerForSteer();
        SimulatedArena.getInstance().addDriveTrainSimulation(swerveSim);

        this.currentSwerveState = new SwerveModuleState();
    }

    @Override
    public void setDesiredSwerveState(SwerveModuleState desiredState) {
        desiredState.optimize(swerveSim.getModules()[index].getSteerAbsoluteFacing());
        desiredState.cosineScale(swerveSim.getModules()[index].getSteerAbsoluteFacing());

        //turn
        if (BasicUtil.numIsInBallparkOf(currentSwerveState.angle.getRadians(), desiredState.angle.getRadians(), SwerveConstants.turnAccuracyToleranceRAD)) {
            turnPID.setSetpoint(MathUtil.inputModulus(desiredState.angle.getRadians(),0, 2 * Math.PI));
            turnMotor.requestVoltage( Volts.of(turnPID.calculate(swerveSim.getModules()[index].getSteerAbsoluteFacing().getRadians())));
        }
        else turnMotor.requestVoltage(Volts.of(0));

        //drive
        if (BasicUtil.numIsInBallparkOf(currentSwerveState.speedMetersPerSecond, desiredState.speedMetersPerSecond, SwerveConstants.driveAccuracyToleranceMPS) && !BasicUtil.numIsInBallparkOf(desiredState.speedMetersPerSecond, 0, SwerveConstants.driveAccuracyToleranceMPS)) {
            drivePID.setSetpoint(desiredState.speedMetersPerSecond);
            DriveMotor.requestVoltage(Volts.of(drivePID.calculate(swerveSim.getModules()[index].getDriveWheelFinalSpeed().in(RadiansPerSecond) * SwerveConstants.wheelRadiusMeters)));
            
        }
        else if (!BasicUtil.numIsInBallparkOf(desiredState.speedMetersPerSecond, 0 , SwerveConstants.driveAccuracyToleranceMPS)) {
            DriveMotor.requestVoltage(Volts.of(driveVolts));
        }
        else DriveMotor.requestVoltage(Volts.of(0));

        //updating information
        currentSwerveState = swerveSim.getModules()[index].getCurrentState();
        driveVolts = DriveMotor.getAppliedVoltage().in(Volts);
        turnVolts = turnMotor.getAppliedVoltage().in(Volts);
    }

    @Override
    public void forceSetVoltage(double turnVolts, double driveVolts) {
        DriveMotor.requestVoltage(Volts.of(0));
        turnMotor.requestVoltage(Volts.of(0));
    }

    @Override
    public void changeModuleDrivePID(String valueToChange, double IncrementAmount) {

        DriveMotor.requestVoltage(Volts.of(0));
        turnMotor.requestVoltage(Volts.of(0));
        switch (valueToChange) {
            case "P":
                ModuleMotorConfig.SIM_DRIVE_P += IncrementAmount;
                break;
            case "I":
                ModuleMotorConfig.SIM_DRIVE_I += IncrementAmount;
                break;
            case "D":
                ModuleMotorConfig.SIM_DRIVE_D += IncrementAmount;
                break;
        
            default:
            return;
        }
    }

    @Override
    public void changeModuleTurnPID(String valueToChange, double IncrementAmount) {
        DriveMotor.requestVoltage(Volts.of(0));
        turnMotor.requestVoltage(Volts.of(0));
        switch (valueToChange) {
            case "P":
                ModuleMotorConfig.SIM_TURN_P += IncrementAmount;
                break;
            case "I":
                ModuleMotorConfig.SIM_TURN_I += IncrementAmount;
                break;
            case "D":
                ModuleMotorConfig.SIM_TURN_D += IncrementAmount;
                break;
        
            default:
            return;
        }
    }

    @Override
    public void periodic() {
        //logging
        GroupLogger.logStructGroup("Swerve Module State", currentSwerveState, SwerveModuleState.struct, index, 4);
        GroupLogger.logDoubleGroup("Swerve Module Turn volts", turnVolts, index, 4);
        GroupLogger.logDoubleGroup("Swerve Module Turn Amps", driveVolts, index, 4);


    }

    @Override
    public SwerveModulePosition getmodulePosition() {
        return new SwerveModulePosition(swerveSim.getModules()[index].getDriveWheelFinalPosition().in(Radians) * SwerveConstants.wheelRadiusMeters, swerveSim.getModules()[index].getSteerAbsoluteFacing());
    }

    



}
