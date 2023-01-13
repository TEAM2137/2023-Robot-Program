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
import frc.robot.library.Constants;
import frc.robot.library.units.Distance2d;
import frc.robot.library.units.Speed2d;

public interface SwerveModule {

    default void setSwerveVelocityModuleState(SwerveModuleState swerveModuleState) {
        setModuleAngle(swerveModuleState.getRotation2d());
        setVelocityDriveSpeed(swerveModuleState.getSpeed2d());
    }

    default void setSwerveDistanceModuleState(SwerveModuleState swerveModuleState) {
        setModuleAngle(swerveModuleState.getRotation2d());
        setDriveDistanceTarget(swerveModuleState.getDistance2d());
    }

    default void setSwerveRawModuleState(SwerveModuleState swerveModuleState) {
        setModuleAngle(swerveModuleState.getRotation2d());
        setRawDriveSpeed(swerveModuleState.getRawPowerValue());
    }

    default void setSwerveModuleState(SwerveModuleState swerveModuleState) {
        setModuleAngle(swerveModuleState.getRotation2d());
        switch(swerveModuleState.getControlType()) {
            case VELOCITY:
                setVelocityDriveSpeed(swerveModuleState.getSpeed2d());
                break;
            case DISTANCE:
                setDriveDistanceTarget(swerveModuleState.getDistance2d());
                break;
            case RAW:
                setRawDriveSpeed(swerveModuleState.getRawPowerValue());
                break;
        }
    }

    void setModuleAngle(Rotation2d angle);
    Rotation2d getModuleAngle();

    void setRawDriveSpeed(double speed);
    void setVelocityDriveSpeed(Speed2d speed);

    double getRawDrivePower();
    Speed2d getDriveVelocity();
    Speed2d getDriveVelocityGoal();

    void setDriveDistanceTarget(Distance2d distance2d);
    Distance2d getDriveDistanceTarget();
    Distance2d getCurrentDrivePosition();

    void setDriveCoastMode(boolean brake);

    void configDrivetrainControlType(Constants.DriveControlType control);

    SwerveModuleState getSwerveModuleState();
}
