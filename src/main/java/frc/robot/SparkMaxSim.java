package frc.robot;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.RoboRioDataJNI;

public class SparkMaxSim extends CANSparkMax{

    private SimDevice sparkSim;
    private SimDouble sparkSIMspeed;

    /**
     * Wrapper class for SparkMAX
     * near identical copy
     * https://github.com/ligerbots/InfiniteRecharge2020/blob/infiniteSimulator/src/main/java/frc/robot/simulation/SparkMaxWrapper.java
     */
    public SparkMaxSim(int deviceID, MotorType type) {
        super(deviceID,type);

        sparkSim = SimDevice.create("Spark Max", deviceID);

        if (sparkSim != null) {
            sparkSIMspeed = sparkSim.createDouble("speed", SimDevice.Direction.kInput, 0.0);
        }
    }

    @Override
    public void set(double speed) {
        if (sparkSim != null) {
            sparkSIMspeed.set(speed);
        } else {
            super.set(speed);
        }
    }

    @Override
    public double get() {
        if (sparkSim != null) {
            return sparkSIMspeed.get();
        } else {
            return super.get();
        }
    }

    @Override
    public void setVoltage(double outputVolts) {
        if (sparkSim != null) {
            //likely incorrect
            set(outputVolts / RoboRioDataJNI.getVInVoltage());
        } else {
            super.setVoltage(outputVolts);
        }        
    }
}