package frc.robot.library;

import frc.robot.library.units.UnitContainers.Rectangle3d;

import java.util.List;

public class Collision {
    /**
     * List of colliding bodies.
     */
    public List<Rectangle3d> colliders;

    public void addCollision(Rectangle3d collider){
        colliders.add(collider);
    }

    public void removeCollision(int index){
        colliders.remove(index);
    }
}
