package frc.robot.library.units;

public interface UnitEnum {

    UnitEnum getPrimaryUnit();
    double getUnitPerPrimaryUnit();
    UnitEnum getFromName(String name);
    UnitUtil.UnitType getUnitType();
}
