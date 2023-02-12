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
import frc.robot.library.units.TranslationalUnits.Distance;
import frc.robot.library.units.TranslationalUnits.Velocity;

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
        optimizeSwerveModuleAngle(swerveModuleState, getModuleAngle());
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

    default void optimizeSwerveModuleAngle(SwerveModuleState state, Rotation2d current) {
        var delta = state.getRotation2d().minus(current);

        if (Math.abs(delta.getDegrees()) > 90.0) {
             state.invertDirection();
        }
    }

    void setModuleAngle(Rotation2d angle);
    Rotation2d getModuleAngle();
    Rotation2d getModuleGoalAngle();

    void setRawDriveSpeed(double speed);
    void setVelocityDriveSpeed(Velocity speed);

    double getRawDrivePower();
    double getCurrentDriveRPM();
    Velocity getDriveVelocity();
    Velocity getDriveVelocityGoal();

    void setDriveDistanceTarget(Distance distance2d);
    Distance getDriveDistanceTarget();
    Distance getCurrentDrivePosition();

    void setDriveCoastMode(boolean brake);

    void configDrivetrainControlType(Constants.DriveControlType control);
    Constants.DriveControlType getDriveControlType();

    SwerveModuleState getSwerveModuleState();

    SwerveModuleState getSwerveModuleAccelerationState(double voltage);

    SwerveModuleState.SwerveModulePositions getSwerveModuleLocation();

    double getDriveMotorVoltage();
}
