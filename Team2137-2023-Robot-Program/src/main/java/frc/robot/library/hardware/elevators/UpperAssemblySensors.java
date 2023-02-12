package frc.robot.library.hardware.elevators;

import com.ctre.phoenix.CANifier;

public interface UpperAssemblySensors {

    CANifier getCANifier();
    boolean getVerticalElevatorZeroSensorState();
}
