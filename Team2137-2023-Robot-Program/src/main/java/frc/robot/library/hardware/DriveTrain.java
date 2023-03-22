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

package frc.robot.library.hardware;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.library.Constants;
import frc.robot.library.units.AngleUnits.Angle;
import frc.robot.library.units.TranslationalUnits.Distance;
import frc.robot.library.units.UnitContainers.CartesianValue;
import frc.robot.library.units.UnitContainers.Pose2d;

public interface DriveTrain {

    default void setSpeed(double speed) {
        setLeftSpeed(speed);
        setRightSpeed(speed);
    }

    void setLeftSpeed(double speed);
    void setRightSpeed(double speed);

    Angle getAngle();

    CartesianValue<Angle> getCartesianAngles();

    void setAngleOffset(Rotation2d offset);

    void configDrivetrainControlType(Constants.DriveControlType control);

    Pose2d<Distance, Angle> getCurrentOdometryPosition();
}
