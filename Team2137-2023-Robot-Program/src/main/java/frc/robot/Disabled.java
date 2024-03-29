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

package frc.robot;

import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.XMLSettingReader;
import frc.robot.library.OpMode;

public class Disabled implements OpMode {

    // private FileLogger logger;
    // private final int mintDebug = 0;

    @Override
    public void init(XMLSettingReader xmlSettingReader, FileLogger fileLogger) {
//        this.logger = new FileLogger(mintDebug, Constants.RobotState.DISABLED);
    }

    @Override
    public void periodic() {
    }

    @Override
    public void end() {
//        logger.writeEvent(0, FileLogger.EventType.Status, "DISABLED Ending");
//        logger.close();
    }
}
