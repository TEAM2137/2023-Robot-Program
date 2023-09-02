package frc.robot.library;

import edu.wpi.first.wpilibj.GenericHID;

/**
 * Add your docs here.
 */
public class Gamepad extends GenericHID {

    public interface ControllerIO {
        String toString();
        int getPort();
    }

    public enum Button implements ControllerIO {
        kA(1, "AButton"),
        kB(2, "BButton"),
        kX(3, "XButton"),
        kY(4, "YButton"),
        kBumperLeft(5, "LeftBumper"),
        kBumperRight(6, "RightBumper"),
        kBack(7, "BackButton"),
        kStart(8, "StartButton"),
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
    public enum Axis implements ControllerIO {
        kLeftX(0, "LeftX"),
        kLeftY(1, "LeftY"),
        kLeftTrigger(2, "Left Trigger"),
        kRightTrigger(3, "Right Trigger"),
        kRightX(4, "RightX"),
        kRightY(5, "RightY");

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
        kUp(0, "DPAD-UP"),
        kRight(90, "DPAD-RIGHT"),
        kDown(180, "DPAD-DOWN"),
        kLeft(270, "DPAD-Left");

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

    public enum Hand {
        kLeft,
        kRight;
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
    public int getDPAD() {
        return getPOV();
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