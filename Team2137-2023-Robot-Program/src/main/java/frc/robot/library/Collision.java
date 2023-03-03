package frc.robot.library;

import frc.robot.library.units.UnitContainers.Point3d;
import frc.robot.library.units.UnitContainers.Rectangle3d;

import java.util.List;

public class Collision {
    /**
     * List of colliding bodies.
     */
    public List<Rectangle3d> colliders;

    public void addCollision(Rectangle3d collider) {
        colliders.add(collider);
    }

    public void removeCollision(int index) {
        colliders.remove(index);
    }

    public boolean checkCollision(Rectangle3d body, int colliderIndex) {
        Point3d[] bodyPoints = body.getPoints();
        Point3d[] colliderPoints = colliders.get(colliderIndex).getPoints();

        if(
                // Is 'body' inside the collider? (Y only)
                (bodyPoints[0].getYPrimary() >= colliderPoints[0].getYPrimary() &&
                bodyPoints[0].getYPrimary() <= colliderPoints[3].getYPrimary()) ||
                (bodyPoints[3].getYPrimary() >= colliderPoints[0].getYPrimary() &&
                bodyPoints[3].getYPrimary() <= colliderPoints[3].getYPrimary()) ||

                // Is the collider inside 'body' (Y only)
                (colliderPoints[0].getYPrimary() >= bodyPoints[0].getYPrimary() &&
                colliderPoints[0].getYPrimary() <= bodyPoints[3].getYPrimary()) ||
                (colliderPoints[3].getYPrimary() >= bodyPoints[0].getYPrimary() &&
                colliderPoints[3].getYPrimary() <= bodyPoints[3].getYPrimary())
        ){
            // TODO: Detect XZ collisions here
        }else{
            return false;
        }

        return false;
    }
}
