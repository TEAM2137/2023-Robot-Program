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

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Add your docs here.
 */
public class Gamepad extends GenericHID {

    public interface ControllerIO {
        String toString();
        int getPort();
    }

    public enum Hand {
        kLeft,
        kRight
    }

    public enum Button implements ControllerIO {
        kA(1, "A"),
        kB(2, "B"),
        kX(3, "X"),
        kY(4, "Y"),
        kBumperLeft(5, "Bumper Left"),
        kBumperRight(6, "Bumper Right"),
        kBack(7, "Back"),
        kStart(8, "Start"),
        kStickLeft(9, "Stick Left"),
        kStickRight(10, "Stick Right");

        @SuppressWarnings({"MemberName", "PMD.SingularField"})
        public final int value;
        public final String name;

        Button(int value, String name) {
            this.value = value;
            this.name  = name;
        }

        @Override
        public String toString(){
            return this.name;
        }

        @Override
        public int getPort(){
            return this.value;
        }
    }

    /**
     * Represents an axis on an XboxController.
     */
    public enum Axis implements ControllerIO{
        kLeftX(0, "Left X"),
        kLeftY(1, "Left Y"),
        kLeftTrigger(2, "Left Trigger"),
        kRightTrigger(3, "Right Trigger"),
        kRightX(4, "Right X"),
        kRightY(5, "Right Y");

        @SuppressWarnings({"MemberName", "PMD.SingularField"})
        public final int value;
        public final String name;

        Axis(int value, String name) {
            this.value = value;
            this.name  = name;
        }

        @Override
        public String toString(){
            return this.name;
        }

        @Override
        public int getPort(){
            return this.value;
        }
    }

    /**
     * Represents the D-Pad on an XboxController
     */
    public enum DPad implements ControllerIO{
        kUp(0, "D-Pad Up"),
        kUpRight(45, "D-Pad Up Right"),
        kRight(90, "D-Pad Right"),
        kDownRight(135, "D-Pad Down Right"),
        kDown(180, "D-Pad Down"),
        kDownLeft(225, "D-Pad Down Left"),
        kLeft(270, "D-Pad Left"),
        kUpLeft(315, "D-Pad Up Left");

        @SuppressWarnings({"MemberName", "PMD.SingularField"})
        public final int value;
        public final String name;

        DPad(int value, String name) {
            this.value = value;
            this.name  = name;
        }

        public String toString(){
            return this.name;
        }

        @Override
        public int getPort() {
            return this.value;
        }
    }

    public Gamepad(int port) {
        super(port);
    }

    public double getX(Hand hand) {
        if(hand == Hand.kLeft)
            return getRawAxis(Axis.kLeftX.getPort());
        else
            return getRawAxis(Axis.kRightX.getPort());
    }

    public double getY(Hand hand) {
        if(hand == Hand.kLeft)
            return getRawAxis(Axis.kLeftY.getPort());
        else
            return getRawAxis(Axis.kRightY.getPort());
    }

    public double getX(Hand hand, double deadbandWidth) {
        return applyDeadBand(getX(hand), deadbandWidth);
    }

    public double getY(Hand hand, double deadbandWidth) {
        return applyDeadBand(getY(hand), deadbandWidth);
    }

    public double getLeftJoyStickX() {
        return getX(Hand.kLeft);
    }
    public double getRightJoyStickX() {
        return getX(Hand.kRight);
    }
    public double getLeftJoyStickY() {
        return getY(Hand.kLeft);
    }
    public double getRightJoyStickY() {
        return getY(Hand.kRight);
    }
    public double getLeftTrigger() {
        return getRawAxis(Axis.kLeftTrigger.getPort());
    }
    public double getRightTrigger() {
        return getRawAxis(Axis.kRightTrigger.getPort());
    }
    public boolean getLeftBumper() {
        return getRawButton(Button.kBumperLeft.getPort());
    }
    public boolean getRightBumper() {
        return getRawButton(Button.kBumperRight.getPort());
    }
    public boolean getLeftStickButton() {
        return getRawButton(Button.kBumperLeft.getPort());
    }
    public boolean getRightStickButton() {
        return getRawButton(Button.kBumperRight.getPort());
    }
    public boolean getButtonA() {
        return getRawButton(Button.kA.getPort());
    }
    public boolean getButtonB() {
        return getRawButton(Button.kB.getPort());
    }
    public boolean getButtonX() {
        return getRawButton(Button.kX.getPort());
    }
    public boolean getButtonY() {
        return getRawButton(Button.kY.getPort());
    }
    public boolean getButtonStart() {
        return getRawButton(Button.kStart.getPort());
    }
    public boolean getButtonBack() {
        return getRawButton(Button.kBack.getPort());
    }
    public boolean getButtonLeftStick() {
        return getRawButton(Button.kStickLeft.getPort());
    }
    public boolean getButtonRightStick() {
        return getRawButton(Button.kStickRight.getPort());
    }
    public boolean getDPadUp() {
        return getPOV() == DPad.kUp.value;
    }
    public boolean getDPadUpRight() {
        return getPOV() == DPad.kUpRight.value;
    }
    public boolean getDPadRight() {
        return getPOV() == DPad.kRight.value;
    }
    public boolean getDPadDownRight() {
        return getPOV() == DPad.kDownRight.value;
    }
    public boolean getDPadDown() {
        return getPOV() == DPad.kDown.value;
    }
    public boolean getDPadDownLeft() {
        return getPOV() == DPad.kDownLeft.value;
    }
    public boolean getDPadLeft() {
        return getPOV() == DPad.kLeft.value;
    }
    public boolean getDPadUpLeft() {
        return getPOV() == DPad.kUpLeft.value;
    }

    public static double applyDeadBand(double value, double deadBandWidth){
        if(Math.abs(value) < deadBandWidth)
            return 0;
        else {
            if (value > 0.0) {
                return (value - deadBandWidth) / (1.0 - deadBandWidth);
            } else {
                return (value + deadBandWidth) / (1.0 - deadBandWidth);
            }
        }
    }
}