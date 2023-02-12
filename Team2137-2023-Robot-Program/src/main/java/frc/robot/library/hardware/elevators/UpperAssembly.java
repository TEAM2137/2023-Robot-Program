package frc.robot.library.hardware.elevators;

import com.ctre.phoenix.CANifier;
import frc.robot.functions.io.xmlreader.objects.CANifierObj;
import frc.robot.library.units.Number;
import org.w3c.dom.Element;

import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.EntityGroup;

public class UpperAssembly extends EntityGroup implements UpperAssemblySensors {

    private FileLogger logger;

    private final CANifierObj mCANifierObj;
    private final Number mPWMChannelVerticalSensor;

    private CANifier mCANifier;

    public UpperAssembly(Element element, EntityGroup parent, FileLogger fileLogger) {
        super(element, parent, fileLogger);
        logger = fileLogger;

        mCANifierObj = (CANifierObj) getEntity("SensorCANifier");
        mPWMChannelVerticalSensor = (Number) getEntity("VerticalSensorSlot");

        init();
    }

    public void init() {
        mCANifier = new CANifier(mCANifierObj.getID());
        mCANifier.configFactoryDefault();
    }

    @Override
    public void periodic() {

    }

    @Override
    public CANifier getCANifier() {
        return null;
    }

    @Override
    public boolean getVerticalElevatorZeroSensorState() {
        return mCANifier.getGeneralInput(CANifier.GeneralPin.LIMF);
    }
}
