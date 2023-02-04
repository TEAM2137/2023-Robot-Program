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

package frc.robot.library.hardware.deadReckoning;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.EntityGroup;
import frc.robot.functions.io.xmlreader.objects.Encoder;
import frc.robot.library.Constants;
import frc.robot.library.units.Distance;
import frc.robot.library.units.Velocity;
import org.ejml.simple.SimpleMatrix;
import org.w3c.dom.Element;

import static frc.robot.library.units.Units.Unit.*;


public class DeadWheelActiveTracking extends EntityGroup {

    private Encoder xEncoderObj;
    private Encoder yEncoderObj;

    private final CANCoder xCANCoder;
    private final CANCoder yCANCoder;

    private Distance wheelDiameter;

    private double xConversionFactor = 1; //Counts for every inch
    private double yConversionFactor = 1; //Counts for every inch

    private final Distance previousXReading = new Distance(0, FOOT);
    private final Distance previousYReading = new Distance(0, FOOT);

    private Distance xValue = new Distance(0, FOOT);
    private Distance yValue = new Distance(0, FOOT);

    private Velocity xSpeed = new Velocity(0, FEET_PER_SECOND);
    private Velocity ySpeed = new Velocity(0, FEET_PER_SECOND);

    public DeadWheelActiveTracking(Element element, EntityGroup parent, FileLogger fileLogger) {
        super(element, parent, fileLogger);

        xEncoderObj = (Encoder) parent.getEntity("xCoder");
        yEncoderObj = (Encoder) parent.getEntity("yCoder");

        xCANCoder = new CANCoder(xEncoderObj.getID());
        yCANCoder = new CANCoder(yEncoderObj.getID());

        wheelDiameter = (Distance) parent.getEntity("WheelDiameter");

        initialize();
        this.setOnImplementCallback(this::initialize);
    }

    public void initialize() {
        xCANCoder.configFactoryDefault();
        xCANCoder.configMagnetOffset(xEncoderObj.getOffset());
        xCANCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
        xCANCoder.configSensorDirection(xEncoderObj.inverted());

        yCANCoder.configFactoryDefault();
        yCANCoder.configMagnetOffset(yEncoderObj.getOffset());
        yCANCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
        yCANCoder.configSensorDirection(yEncoderObj.inverted());

        xConversionFactor = (wheelDiameter.getValue(FOOT) * Math.PI) / 360;// / 4096.0;
        yConversionFactor = (wheelDiameter.getValue(FOOT) * Math.PI) / 360;// / 4096.0;
    }

    public void updateTracking(Rotation2d robotHeading) {
//        Distance changeInXDistance = getCurrentXReading().minus(previousXReading);
//        Distance changeInYDistance = getCurrentYReading().minus(previousYReading);
//
//        SimpleMatrix matrix = Constants.convertFrame(robotHeading.getDegrees(), Constants.createFrameMatrix(changeInXDistance, changeInYDistance, robotHeading));

//        xValue.mutablePlus(matrix.get(0,0));
//        yValue.mutablePlus(matrix.get(1,0));
//
//        xSpeed.mutableChange(xCANCoder.getVelocity());
//        ySpeed.mutableChange(yCANCoder.getVelocity());
    }

    public Distance getGlobalXReading() {
        return xValue;
    }

    public Distance getGlobalYReading() {
        return yValue;
    }

    public Velocity getXVelocity() {
        return xSpeed;
    }

    public Velocity getYVelocity() {
        return ySpeed;
    }

    private Distance getCurrentXReading() {
        return new Distance(xCANCoder.getPosition() / xConversionFactor, INCH);
    }

    private Distance getCurrentYReading() {
        return new Distance(yCANCoder.getPosition() / yConversionFactor, INCH);
    }
}
