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

package frc.robot.library;

import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.XMLSettingReader;
import frc.robot.functions.io.xmlreader.XMLStepReader;

public interface OpMode {

    /**
     * Runs once when the robot state changed.
     */
    void init(XMLSettingReader xmlSettingReader, XMLStepReader xmlStepReader, FileLogger fileLogger);

    /**
     * Repeats until robot state is changed.
     */
    void periodic();

    /**
     * Runs once after the robot state has changed.
     */
    void end();
}
