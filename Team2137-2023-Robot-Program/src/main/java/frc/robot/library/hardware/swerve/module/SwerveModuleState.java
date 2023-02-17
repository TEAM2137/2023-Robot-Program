//              Copyright 2022 Wyatt Ashley
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
package frc.robot.library.hardware.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.functions.io.FileLogger;
import frc.robot.library.Constants;
import frc.robot.library.units.*;
import frc.robot.library.units.AngleUnits.AngularAcceleration;
import frc.robot.library.units.TranslationalUnits.Acceleration;
import frc.robot.library.units.TranslationalUnits.Distance;
import frc.robot.library.units.TranslationalUnits.Velocity;
import frc.robot.library.units.Number;
import frc.robot.library.units.UnitContainers.Vector2d;

import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.FOOT;
import static frc.robot.library.units.TranslationalUnits.Velocity.VelocityUnits.FEET_PER_SECOND;

public class SwerveModuleState {
    public enum SwerveModulePositions {
        LEFT_FRONT ("LeftFront"),
        LEFT_BACK ("LeftBack"),
        RIGHT_FRONT ("RightFront"),
        RIGHT_BACK ("RightBack");

        private final String name;

        SwerveModulePositions(String _name) {
            name = _name;
        }

        public String toString() {
            return name;
        }

        public static SwerveModulePositions getPositionFromString(String value) {

            if(value.equalsIgnoreCase("leftfront"))
                return LEFT_FRONT;
            if(value.equalsIgnoreCase("leftback"))
                return LEFT_BACK;
            if(value.equalsIgnoreCase("rightfront"))
                return RIGHT_FRONT;
            if(value.equalsIgnoreCase("rightback"))
                return RIGHT_BACK;

            return null;
        }
    }

    private Rotation2d rotation2d;
    private AngularAcceleration rotationalAccel;

    private Acceleration acceleration;
    private Velocity speed2d;
    private Distance distance2d;
    private Double rawPowerValue;

    private final Constants.DriveControlType controlType;
    private final SwerveModulePositions position;

    public SwerveModuleState(Acceleration _accel, AngularAcceleration _rotation2d, SwerveModulePositions pos) {
        rotationalAccel = _rotation2d;
        acceleration = _accel;
        controlType = Constants.DriveControlType.ACCELERATION;
        position = pos;
    }

    public SwerveModuleState(Velocity _speed, Rotation2d _rotation2d, SwerveModulePositions pos) {
        rotation2d = _rotation2d;
        speed2d = _speed;
        controlType = Constants.DriveControlType.VELOCITY;
        position = pos;
    }

    public SwerveModuleState(Distance _distance, Rotation2d _rotation2d, SwerveModulePositions pos) {
        rotation2d = _rotation2d;
        distance2d = _distance;
        controlType = Constants.DriveControlType.DISTANCE;
        position = pos;
    }

    public SwerveModuleState(double _rawPowerValue, Rotation2d _rotation2d, SwerveModulePositions pos) {
        rotation2d = _rotation2d;
        rawPowerValue = _rawPowerValue;
        controlType = Constants.DriveControlType.RAW;
        position = pos;
    }

    public SwerveModuleState(Unit<?, ?> _number, Rotation2d _angle, SwerveModulePositions pos) {
        if(_number instanceof Velocity) {
            speed2d = (Velocity) _number;
            controlType = Constants.DriveControlType.VELOCITY;
        } else if(_number instanceof Distance) {
            distance2d = (Distance) _number;
            controlType = Constants.DriveControlType.DISTANCE;
        } else {
            rawPowerValue = _number.getValueInPrimaryUnit();
            controlType = Constants.DriveControlType.RAW;
        }

        rotation2d = _angle;
        position = pos;
    }

    public SwerveModuleState(Unit<?, ? extends UnitEnum> _number, Unit<?, ? extends UnitEnum> _angle, SwerveModulePositions pos) {
        if(_number instanceof Velocity) {
            speed2d = (Velocity) _number;
            controlType = Constants.DriveControlType.VELOCITY;
        } else if(_number instanceof Distance) {
            distance2d = (Distance) _number;
            controlType = Constants.DriveControlType.DISTANCE;
        } else if (_number instanceof Acceleration) {
            acceleration = (Acceleration) _number;
            controlType = Constants.DriveControlType.ACCELERATION;
        } else {
            rawPowerValue = _number.getValueInPrimaryUnit();
            controlType = Constants.DriveControlType.RAW;
        }

        if(_angle instanceof AngularAcceleration) {
            rotationalAccel = (AngularAcceleration) _angle;
        } else {
            rotation2d = Rotation2d.fromRadians(_angle.getValueInPrimaryUnit());
        }
        position = pos;
    }

    public SwerveModuleState(Vector2d<Number> vect, SwerveModulePositions pos) {
        this(vect.getMagnitude(), vect.getAngle(), pos);
    }

    public Rotation2d getRotation2d() {
        return rotation2d;
    }

    public AngularAcceleration getRotationalAccel() {
        return rotationalAccel;
    }

    public Velocity getSpeed2d() {
        if(speed2d != null)
            return speed2d;
        else
            throw new NullPointerException("Can not give Velocity when State is a different Control Type");
    }

    public Distance getDistance2d() {
        if (distance2d != null)
            return distance2d;
        else
            throw new NullPointerException("Can not give Raw Power when State is a different Control Type");
    }

    public Acceleration getAcceleration() {
        if(acceleration != null)
            return acceleration;
        else
            throw new NullPointerException("Can not give Acceleration when State is a different Control Type");
    }

    public double getRawPowerValue() {
        if(rawPowerValue != null)
            return rawPowerValue;
        else
            throw new NullPointerException("Can not give Raw Power when State is a different Control Type");
    }

    public void invertDirection() {
        rotation2d = rotation2d.rotateBy(Rotation2d.fromDegrees(180.0));

        switch(getControlType()) {
            case RAW:
                rawPowerValue = -rawPowerValue;
                break;
            case DISTANCE:
                distance2d = distance2d.times(-1.0);
                break;
            case VELOCITY:
                speed2d = speed2d.times(-1.0);
                break;
        }
    }

    public Constants.DriveControlType getControlType() {
        return controlType;
    }

    public SwerveModulePositions getPosition() { return position; }

    @Override
    public String toString() {
        switch (getControlType()) {
            case DISTANCE:
                return getPosition().toString() + " Distance: " + getDistance2d().getValue(FOOT) + "ft Angle: " + getRotation2d().getRadians();
            case RAW:
                return getPosition().toString() + " RAW: " + getRawPowerValue() + " Angle: " + getRotation2d().getDegrees();
            case VELOCITY:
                return getPosition().toString() + " Velocity: " + getSpeed2d().getValue(FEET_PER_SECOND) + "ft/s Angle: " + getRotation2d().getRadians();
        }
        return "";
    }

    public void writeToFileLoggerReplayFormat(FileLogger fileLogger) {
        fileLogger.setTag("");

        String key = "";
        switch(getPosition()) {
            case LEFT_FRONT:
                key = Constants.StandardFileLoggerKeys.LEFT_FRONT_SWERVE_STATE.getKey();
                break;
            case LEFT_BACK:
                key = Constants.StandardFileLoggerKeys.LEFT_BACK_SWERVE_STATE.getKey();
                break;
            case RIGHT_FRONT:
                key = Constants.StandardFileLoggerKeys.RIGHT_FRONT_SWERVE_STATE.getKey();
                break;
            case RIGHT_BACK:
                key = Constants.StandardFileLoggerKeys.RIGHT_BACK_SWERVE_STATE.getKey();
                break;
        }
        
        StringBuilder builder = new StringBuilder();
        builder.append("Q~").append(key).append("~");
        builder.append(getRotation2d().getDegrees()).append(" ");

        switch(getControlType()) {
            case DISTANCE:
                builder.append(getDistance2d().getValue(FOOT));
                break;
            case VELOCITY:
                builder.append(getSpeed2d().getValue(FEET_PER_SECOND));
                break;
            case RAW:
                builder.append(getRawPowerValue());
                break;
        }

        builder.append(" ");

        fileLogger.writeLine(builder.toString());
    }
}
