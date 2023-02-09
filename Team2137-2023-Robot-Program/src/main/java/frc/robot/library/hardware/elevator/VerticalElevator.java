package frc.robot.library.hardware.elevator;

import com.ctre.phoenix.CANifier;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.EntityGroup;
import frc.robot.functions.io.xmlreader.XMLSettingReader;
import frc.robot.functions.io.xmlreader.data.PID;
import frc.robot.functions.io.xmlreader.objects.Motor;
import frc.robot.library.Constants;
import frc.robot.library.units.TranslationalUnits.Distance;
import frc.robot.library.units.Number;
import org.w3c.dom.Element;

import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.FOOT;
import static frc.robot.library.units.TranslationalUnits.Distance.DistanceUnits.INCH;

public class VerticalElevator extends EntityGroup {

    public enum PresetElevatorPositions {
        CONE_LEVEL_1 (new Distance(1, FOOT)),
        CONE_LEVEL_2 (new Distance(2, FOOT)),
        CONE_LEVEL_3 (new Distance(3, FOOT)),
        CUBE_LEVEL_1 (new Distance(0.5, FOOT)),
        CUBE_LEVEL_2 (new Distance(1.5, FOOT)),
        CUBE_LEVEL_3 (new Distance(2.5, FOOT));

        final Distance position;

        PresetElevatorPositions(Distance pos) {
            position = pos;
        }

        public Distance getPosition() {
            return position;
        }
    }

    private final Motor mLiftMotor1Obj;
    private final Motor mLiftMotor2Obj;
    private final PID mLiftPIDObj;

    private CANSparkMax mLiftMotor1;
    private CANSparkMax mLiftMotor2;
    private RelativeEncoder mLiftEncoder;
    private SparkMaxPIDController mLiftPIDController;

    private Distance mGoalPosition;
    private Distance mSpoolDiameter;
    private double mCountsPerInch;

    private Constants.StepState mHomingStepState;

    /**
     * Takes in a part of the xml file and parses it into variables and subtypes using recursion
     *
     * @param element    - Portion of the XML File
     * @param parent
     * @param fileLogger
     */
    public VerticalElevator(Element element, EntityGroup parent, FileLogger fileLogger) {
        super(element, parent, fileLogger);

        mLiftMotor1Obj = (Motor) getEntity("Lift Motor 1");
        mLiftMotor2Obj = (Motor) getEntity("Lift Motor 2");
        mLiftPIDObj = (PID) getEntity("Lift PID");

        init();
    }

    public void init() {
        mLiftMotor1 = new CANSparkMax(mLiftMotor1Obj.getID(), mLiftMotor1Obj.getMotorType().getREVType());
        mLiftMotor1.restoreFactoryDefaults();
        mLiftMotor1.setInverted(mLiftMotor1Obj.inverted());

        mLiftEncoder = mLiftMotor1.getEncoder();

        mSpoolDiameter = new Distance(((Number) XMLSettingReader.settingsEntityGroup.getEntity("Elevator-SpoolDiameter")).getValue(), INCH);
        mCountsPerInch = (mSpoolDiameter.getValue(INCH) * Math.PI) / mLiftEncoder.getCountsPerRevolution();

        mLiftEncoder.setPositionConversionFactor(1 / mCountsPerInch);
        //mLiftMotor1.setSmartCurrentLimit(mLiftMotor1Obj.getCurrentLimit()); Only if needed

        mLiftMotor2 = new CANSparkMax(mLiftMotor2Obj.getID(), mLiftMotor2Obj.getMotorType().getREVType());
        mLiftMotor2.restoreFactoryDefaults();
        mLiftMotor2.setInverted(mLiftMotor2Obj.inverted());
//        mLiftMotor2.setSmartCurrentLimit(mLiftMotor2Obj.getCurrentLimit());
        mLiftMotor2.follow(mLiftMotor1, mLiftMotor1Obj.inverted() != mLiftMotor2Obj.inverted());

        mLiftPIDController = mLiftMotor1.getPIDController();
        mLiftPIDController.setP(mLiftPIDObj.getP());
        mLiftPIDController.setI(mLiftPIDObj.getI());
        mLiftPIDController.setD(mLiftPIDObj.getD());
        mLiftPIDController.setFF(mLiftPIDObj.getFF());
    }

    @Override
    public void periodic() {
        switch (mHomingStepState) {
            case STATE_INIT:
                mLiftMotor1.set(-0.5);
                break;
            case STATE_RUNNING:

                break;
        }
    }

    public void homeElevator() {
        mHomingStepState = Constants.StepState.STATE_INIT;
    }

    public void setPosition(Distance distance) {
        mLiftPIDController.setReference(distance.getValue(INCH), CANSparkMax.ControlType.kPosition);
    }

    public void setPosition(PresetElevatorPositions pos) {
        setPosition(pos.getPosition());
    }
}
