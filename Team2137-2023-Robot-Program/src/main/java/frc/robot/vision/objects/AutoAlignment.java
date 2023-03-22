package frc.robot.vision.objects;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.functions.io.xmlreader.EntityImpl;
import frc.robot.functions.io.xmlreader.data.PID;
import frc.robot.functions.io.xmlreader.data.Step;
import frc.robot.functions.io.xmlreader.data.mappings.PersistentCommand;
import frc.robot.library.Constants;
import frc.robot.library.hardware.DriveTrain;
import frc.robot.library.hardware.swerve.module.SwerveModuleState;
import frc.robot.library.units.AngleUnits.Angle;
import frc.robot.library.units.AngleUnits.AngularVelocity;
import frc.robot.library.units.TranslationalUnits.Distance;
import frc.robot.library.units.TranslationalUnits.Velocity;
import frc.robot.library.units.UnitContainers.Pose2d;
import frc.robot.library.units.UnitContainers.Vector2d;
import frc.robot.vision.limelight.ReflectiveTape;
import org.opencv.core.Point;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.library.hardware.swerve.SwerveDrivetrain;
import frc.robot.library.units.Number;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;


import static frc.robot.library.units.AngleUnits.Angle.AngleUnits.DEGREE;
import static frc.robot.library.units.AngleUnits.AngularVelocity.AngularVelocityUnits.DEGREE_PER_SECOND;
import static frc.robot.library.units.TranslationalUnits.Velocity.VelocityUnits.FEET_PER_SECOND;

public class AutoAlignment extends EntityImpl {

    private PIDController thetaController, yTranslationalController, xTranslationalController;
    private final SwerveDrivetrain drivetrain;

    /**
     * Creates a new vision pid with an offset defaulted to 0
     */
    public AutoAlignment(Element element) {
        super(element);

        PID thetaValues, yTranslationValues, xTranslationValues;

        NodeList tmpList = element.getElementsByTagName("PID");

        for (int i = 0; i < tmpList.getLength(); i++) {
            String name = getNodeOrAttribute(((Element) tmpList.item(i)), "Name", null);

            if(name.equalsIgnoreCase("ThetaController")) {
                thetaValues = new PID((Element) tmpList.item(i));

                thetaController = new PIDController(thetaValues.getP(), thetaValues.getI(), thetaValues.getD());
                thetaController.enableContinuousInput(0, 360);
                thetaController.setTolerance(2);
            } else if(name.equalsIgnoreCase("YController")) {
                yTranslationValues = new PID((Element) tmpList.item(i));

                yTranslationalController = new PIDController(yTranslationValues.getP(), yTranslationValues.getI(), yTranslationValues.getD());
            } else if(name.equalsIgnoreCase("XController")) {
                xTranslationValues = new PID((Element) tmpList.item(i));

                xTranslationalController = new PIDController(xTranslationValues.getP(), xTranslationValues.getI(), xTranslationValues.getD());
            }
        }

        drivetrain = (SwerveDrivetrain) Robot.robotEntityGroup.getEntityGroupByType("DriveTrain");

        Robot.subSystemCommandList.put("ScoringAutoAlignment", this::AutoAlign);
    }

    public Pose2d<Velocity, AngularVelocity> calculateTranslation(Angle theta, Angle tx, Distance currentX) {
        AngularVelocity omega = new AngularVelocity(-thetaController.calculate(theta.getValue(Angle.AngleUnits.DEGREE)), DEGREE_PER_SECOND);
        Velocity yVel  = new Velocity(yTranslationalController.calculate(tx.getValue(Angle.AngleUnits.DEGREE)), FEET_PER_SECOND);
        Velocity xVel  = new Velocity(xTranslationalController.calculate(currentX.getValue(Distance.DistanceUnits.FOOT)), FEET_PER_SECOND);

        return new Pose2d<>(xVel, yVel, omega);
    }


    @PersistentCommand
    public void AutoAlign(Step step) {
        if (step.getStepState() == Constants.StepState.STATE_INIT) {
            if(step.getParm(1) > 0.5) {
                step.changeStepState(Constants.StepState.STATE_RUNNING);

                thetaController.setSetpoint(0);
            }
        }

        if(step.getStepState() == Constants.StepState.STATE_RUNNING) {
            //Check if driver is still holding the command
            if(step.getParm(1) <= 0.5) {
                //Change step state to INIT (Canceling Current Step)
                step.changeStepState(Constants.StepState.STATE_INIT);

                drivetrain.setSpeed(0);
                return;
            }

            int targetCounts = step.getParm(1).intValue();
            Angle tx = new Angle(step.getParm(2), DEGREE);
            Angle robotAngle = drivetrain.getAngle();
            Distance xCurrent = drivetrain.getCurrentOdometryPosition().getX();

            if(targetCounts > 0) {
                Pose2d<Velocity, AngularVelocity> calculated = calculateTranslation(robotAngle, tx, xCurrent);
                SwerveModuleState[] states = drivetrain.calculateSwerveMotorSpeedsFieldCentric(calculated.getX(), calculated.getY(), calculated.getTheta());

                drivetrain.setSwerveModuleStates(states);
            }
        }
    }
}
