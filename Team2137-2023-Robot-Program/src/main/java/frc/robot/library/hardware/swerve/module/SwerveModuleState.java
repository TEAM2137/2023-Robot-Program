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
import frc.robot.library.units.Distance2d;
import frc.robot.library.units.Speed2d;
import frc.robot.library.units.Time2d;

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

    private final Rotation2d rotation2d;

    private Speed2d speed2d;
    private Distance2d distance2d;
    private Double rawPowerValue;

    private final Constants.DriveControlType controlType;
    private final SwerveModulePositions position;

    public SwerveModuleState(Speed2d _speed, Rotation2d _rotation2d, SwerveModulePositions pos) {
        rotation2d = _rotation2d;
        speed2d = _speed;
        controlType = Constants.DriveControlType.VELOCITY;
        position = pos;
    }

    public SwerveModuleState(Distance2d _distance, Rotation2d _rotation2d, SwerveModulePositions pos) {
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

    public Rotation2d getRotation2d() {
        return rotation2d;
    }

    public Speed2d getSpeed2d() {
        if(speed2d != null)
            return speed2d;
        else
            throw new NullPointerException("Can not give Raw Power when State is a different Control Type");
    }

    public Distance2d getDistance2d() {
        if (distance2d != null)
            return distance2d;
        else
            throw new NullPointerException("Can not give Raw Power when State is a different Control Type");
    }

    public double getRawPowerValue() {
        if(rawPowerValue != null)
            return rawPowerValue;
        else
            throw new NullPointerException("Can not give Raw Power when State is a different Control Type");
    }

    public Constants.DriveControlType getControlType() {
        return controlType;
    }

    public SwerveModulePositions getPosition() { return position; }

    @Override
    public String toString() {
        switch (getControlType()) {
            case DISTANCE:
                return "Distance: " + getDistance2d().getValue(Distance2d.DistanceUnits.FEET) + "ft Angle: " + getRotation2d().getRadians();
            case RAW:
                return "RAW: " + getRawPowerValue() + " Angle: " + getRotation2d().getRadians();
            case VELOCITY:
                return "Velocity: " + getSpeed2d().getValue(Distance2d.DistanceUnits.FEET, Time2d.TimeUnits.SECONDS) + "ft/s Angle: " + getRotation2d().getRadians();
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
                builder.append(getDistance2d().getValue(Distance2d.DistanceUnits.FEET));
                break;
            case VELOCITY:
                builder.append(getSpeed2d().getValue(Distance2d.DistanceUnits.FEET, Time2d.TimeUnits.SECONDS));
                break;
            case RAW:
                builder.append(getRawPowerValue());
                break;
        }

        builder.append(" ");

        fileLogger.writeLine(builder.toString());
    }
}
