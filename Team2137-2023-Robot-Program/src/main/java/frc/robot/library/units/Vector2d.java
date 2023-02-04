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

package frc.robot.library.units;

public class Vector2d<T extends Number> {
    T x;
    T y;

    public Vector2d(T _x, T _y) {
        x = _x;
        y = _y;
    }

    public Vector2d normalize() {
        double length = Math.sqrt(Math.pow(x.getValue(), 2) + Math.pow(y.getValue(), 2));

        T newX = (T) x.clone();
        newX.setValue(x.getValue() / length);
        T newY = (T) y.clone();
        newY.setValue(y.getValue() / length);

        return new Vector2d(newX, newY);
    }

    public Vector2d scale(double scale) {

        T newX = (T) x.clone();
        newX.setValue(x.getValue() * scale);
        T newY = (T) y.clone();
        newY.setValue(y.getValue() * scale);

        return new Vector2d(newX, newY);
    }

    public Angle getAngle() {
        return new Angle(Math.atan2(getY().getValueInPrimaryUnit(), getX().getValueInPrimaryUnit()), Units.Unit.RADIAN);
    }

    public T getX() {
        return x;
    }

    public T getY() {
        return y;
    }

    public T getMagnitude() {
        double d = Math.sqrt(Math.pow(getX().getValueInPrimaryUnit(), 2) + Math.pow(getY().getValueInPrimaryUnit(), 2));
        return (T) Number.create(d, getX().getPrimaryUnit());
    }

//    /**
//     * Will return in Inches
//     * @param translation2d
//     * @return
//     */
//    public Translation2d applyVector(Translation2d translation2d) {
//        return new Translation2d(translation2d.getX() + this.x, translation2d.getY() + this.y);
//    }

    public double slope() {
        return y.getValue() / x.getValue();
    }
}
