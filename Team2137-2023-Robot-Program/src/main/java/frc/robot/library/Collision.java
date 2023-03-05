package frc.robot.library;

import frc.robot.library.units.UnitContainers.Point3d;
import frc.robot.library.units.UnitContainers.Rectangle3d;

import java.util.List;

public class Collision {
    /**
     * List of colliding bodies.
     */
    public List<Rectangle3d> colliders;

    /**
     * Adds a collider defined by the Rectangle3d passed.
     * @param collider Rectangle3d defining the collider.
     */
    public void addCollider(Rectangle3d collider) {
        colliders.add(collider);
    }

    /**
     * Removes the collider at the given index.
     * @param index Index of the collider to remove.
     */
    public void removeCollider(int index) {
        colliders.remove(index);
    }

    /**
     * Checks if the passed collider collides with the collider at colliderIndex.
     * @param body Rectangle3d defining the collider to be checked.
     * @param colliderIndex Index of the collider that is to be checked against.
     * @return Returns true if a collision is detected, false if no collisions detected.
     */
    public boolean checkCollision(Rectangle3d body, int colliderIndex) {
        Point3d[] bodyPoints = body.getPoints();
        Point3d[] colliderPoints = colliders.get(colliderIndex).getPoints();

        if(
                // Is 'body' inside the collider? (Y only)
                (bodyPoints[0].getYPrimary() >= colliderPoints[0].getYPrimary() &&
                bodyPoints[0].getYPrimary() <= colliderPoints[4].getYPrimary()) ||
                (bodyPoints[4].getYPrimary() >= colliderPoints[0].getYPrimary() &&
                bodyPoints[4].getYPrimary() <= colliderPoints[4].getYPrimary()) ||

                // Is the collider inside 'body' (Y only)
                (colliderPoints[0].getYPrimary() >= bodyPoints[0].getYPrimary() &&
                colliderPoints[0].getYPrimary() <= bodyPoints[4].getYPrimary()) ||
                (colliderPoints[4].getYPrimary() >= bodyPoints[0].getYPrimary() &&
                colliderPoints[4].getYPrimary() <= bodyPoints[4].getYPrimary())
        ){
            // Y CHECK SUCCESS: Check if the shapes overlap on XZ plane
            for(int i = 0; i < 4; i++){ // Iterate through each point on the base of 'body'
                double px = bodyPoints[i].getXPrimary();
                double pz = bodyPoints[i].getZPrimary();

                for (int s = 0; s < 4; s++) { // Check each side of the rectangle
                    double x1 = colliderPoints[s].getXPrimary();
                    double x2 = colliderPoints[s > 2 ? 0 : s + 1].getXPrimary();

                    double z1 = colliderPoints[s].getZPrimary();
                    double z2 = colliderPoints[s > 2 ? 0 : s + 1].getZPrimary();

                    if((x2 - x1) * (pz - z1) - (px - x1) * (z2 - z1) >= 0){
                        return true; // Point is touching the collider, collision detected
                    }
                }
            }

            // SECONDARY CHECK FOR IF COLLIDER IS SMALLER THAN BODY
            for(int i = 0; i < 4; i++){ // Iterate through each point on the base of the collider
                double px = colliderPoints[i].getXPrimary();
                double pz = colliderPoints[i].getZPrimary();

                for (int s = 0; s < 4; s++) { // Check each side of the rectangle
                    double x1 = bodyPoints[s].getXPrimary();
                    double x2 = bodyPoints[s > 2 ? 0 : s + 1].getXPrimary();

                    double z1 = bodyPoints[s].getZPrimary();
                    double z2 = bodyPoints[s > 2 ? 0 : s + 1].getZPrimary();

                    if((x2 - x1) * (pz - z1) - (px - x1) * (z2 - z1) >= 0){
                        return true; // Point is touching the body, collision detected
                    }
                }
            }

            return false; // No points collided.

        }else{
            return false; // Failed Y check; impossible to be colliding.
        }
    }
}
