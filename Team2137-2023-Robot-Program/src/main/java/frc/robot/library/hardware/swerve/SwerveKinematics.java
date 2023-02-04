package frc.robot.library.hardware.swerve;

import edu.wpi.first.math.Num;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.library.hardware.swerve.module.SwerveModule;
import frc.robot.library.hardware.swerve.module.SwerveModuleState;
import frc.robot.library.units.*;
import frc.robot.library.units.Number;
import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;
import java.util.Set;

public class SwerveKinematics<T extends Number> {
    private static SimpleMatrix forwardKinematicForm;
    private static SimpleMatrix inverseKinematicForm;

    public SwerveKinematics(Distance robotXLength, Distance robotYLength) {
        inverseKinematicForm = new SimpleMatrix(new double[][] {
                //Left Front
                new double[] { 1, 0, -robotYLength.getValue(Units.Unit.METER)},
                new double[] { 0, 1, -robotXLength.getValue(Units.Unit.METER)},
                //Right Front
                new double[] { 1, 0, -robotYLength.getValue(Units.Unit.METER)},
                new double[] { 0, 1, robotXLength.getValue(Units.Unit.METER)},
                //Left Back
                new double[] { 1, 0, robotYLength.getValue(Units.Unit.METER)},
                new double[] { 0, 1, -robotYLength.getValue(Units.Unit.METER)},
                //Right Back
                new double[] { 1, 0, robotYLength.getValue(Units.Unit.METER)},
                new double[] { 0, 1, robotXLength.getValue(Units.Unit.METER)},
        });

        forwardKinematicForm = inverseKinematicForm.pseudoInverse();
    }

    private SimpleMatrix getForwardKinematic(SimpleMatrix moduleComponentVelocity) {
        return forwardKinematicForm.mult(moduleComponentVelocity);
    }

    private SimpleMatrix getInverseKinematic(SimpleMatrix robotComponentVelocity) {
        return inverseKinematicForm.mult(robotComponentVelocity);
    }

    //Returns Left Front, Right Front, Right Back, Left Back
    public SwerveModuleState[] getSwerveModuleState(double x, double y, double omega) {

        SimpleMatrix desiredComponentsMatrix = new SimpleMatrix(new double[][] {
                new double[] { x },
                new double[] { y },
                new double[] { omega }
        });

        SimpleMatrix states = getInverseKinematic(desiredComponentsMatrix);

        SwerveModuleState[] moduleStates = new SwerveModuleState[4];
        try {
            moduleStates[0] = getStateValue(states.get(0, 0), states.get(1, 0), Units.Unit.SCALAR, SwerveModuleState.SwerveModulePositions.LEFT_FRONT);
            moduleStates[1] = getStateValue(states.get(2, 0), states.get(3, 0), Units.Unit.SCALAR, SwerveModuleState.SwerveModulePositions.RIGHT_FRONT);
            moduleStates[2] = getStateValue(states.get(4, 0), states.get(5, 0), Units.Unit.SCALAR, SwerveModuleState.SwerveModulePositions.LEFT_BACK);
            moduleStates[3] = getStateValue(states.get(6, 0), states.get(7, 0), Units.Unit.SCALAR, SwerveModuleState.SwerveModulePositions.RIGHT_BACK);
        } catch (Exception e) {
            e.printStackTrace();
        }

        return moduleStates;
    }

    private SwerveModuleState getStateValue(double x, double y, Units.Unit unit, SwerveModuleState.SwerveModulePositions pos) {
        double rad = Math.atan2(y, x);
        double mag = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        Rotation2d offset = Rotation2d.fromDegrees(90);
        return new SwerveModuleState(Number.create(mag, unit), new Angle(rad + offset.getRadians(), Units.Unit.RADIAN), pos);
    }
}
