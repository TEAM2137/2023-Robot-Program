package frc.robot.library.hardware.elevators;

import frc.robot.library.units.TranslationalUnits.Distance;

import java.util.concurrent.Callable;

public interface Elevator {
    void homeElevator();
    void setPosition(Distance distance);
    void setSpeed(double speed);
}
