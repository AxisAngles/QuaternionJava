/* MIT License

Copyright (c) 2022, Donald F Reynolds

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. */

// The goal of this library is to be reasonably minimal and complete
// While also being reasonably numerically stable and efficient
// Every function has been derived or rederived from scratch

public final class Quaternion {
    private static float EULER_TOL = 10000f; // approximately tan(pi/2*0.999)/**
    
    public float w, x, y, z;

    /**
     * Creates a new identity quaternion
     */
    public Quaternion() {
        w = 1f;
        x = 0f;
        y = 0f;
        z = 0f;
    }

    /**
     * Creates an arbitrary quaternion
     */
    public Quaternion(float w, float x, float y, float z) {
        this.w = w;
        this.x = x;
        this.y = y;
        this.z = z;
    }

    /**
     * Creates a copy of a quaternion
     */
    public Quaternion(Quaternion Q) {
        w = Q.w;
        x = Q.x;
        y = Q.y;
        z = Q.z;
    }




    /**
     * Sets this to a quaternion generated from a uniform mapping of the inputs
     * @param r0 0 <= r0 < 1
     * @param r1 0 <= r1 < 1
     * @param r2 0 <= r2 < 1
     * @param r3 0 <= r3 < 1
     * @return this
     */
    public Quaternion fromRandomLocal(float r0, float r1, float r2, float r3) {
        if (r0 == 0f && r1 == 0f) {
            w = 1;
            x = 0;
            y = 0;
            z = 0;
            return this;
        }
        float l0 = (float) Math.log(1f - r0);
        float l1 = (float) Math.log(1f - r1);
        float m0 = (float) Math.sqrt(l0/(l0 + l1));
        float m1 = (float) Math.sqrt(l1/(l0 + l1));
        float c2 = (float) Math.cos(6.2831853f*r2);
        float c3 = (float) Math.cos(6.2831853f*r3);
        float s2 = (float) Math.sin(6.2831853f*r2);
        float s3 = (float) Math.sin(6.2831853f*r3);

        w = m0*c2;
        x = m0*s2;
        y = m1*c3;
        z = m1*s3;

        return this;
    }

    /**
     * @param r0 0 <= r0 < 1
     * @param r1 0 <= r1 < 1
     * @param r2 0 <= r2 < 1
     * @param r3 0 <= r3 < 1
     * @return A new quaternion generated from a uniform mapping of the inputs
     */
    public static Quaternion fromRandom(float r0, float r1, float r2, float r3) {
        Quaternion output = new Quaternion();
        return output.fromRandomLocal(r0, r1, r2, r3);
    }



    // Orthogonalization can be done with R*inverse(sqrt(transpose(R)*R))

    /**
     * Sets this to represent the same rotation as the matrix
     * Non-standard matrix naming (xy means xVector's yComponent)
     * Matrix must be positive unit orthogonal
     * @return this
     */
    public Quaternion fromRotationMatrixLocal(
        float xx, float yx, float zx,
        float xy, float yy, float zy,
        float xz, float yz, float zz
    ) {
        if (yy > -zz && zz > -xx && xx > -yy) {
            w = 1 + xx + yy + zz;
            x = yz - zy;
            y = zx - xz;
            z = xy - yx;
        } else if (xx > yy && xx > zz) {
            w = yz - zy;
            x = 1 + xx - yy - zz;
            y = xy + yx;
            z = xz + zx;
        } else if (yy > zz) {
            w = zx - xz;
            x = xy + yx;
            y = 1 - xx + yy - zz;
            z = yz + zy;
        } else {
            w = xy - yx;
            x = xz + zx;
            y = yz + zy;
            z = 1 - xx - yy + zz;
        }

        // Hold off on unitization until the end
        float inv = 1/(float) Math.sqrt(w*w + x*x + y*y + z*z);
        w *= inv;
        x *= inv;
        y *= inv;
        z *= inv;

        return this;
    }

    /**
     * Sets this to represent the same rotation as the matrix
     * Matrix must be positive unit orthogonal
     * @return this
     */
    public Quaternion fromRotationMatrixLocal(Matrix3f matrix) {
        return this.fromRotationMatrixLocal(
            matrix.m00, matrix.m01, matrix.m02,
            matrix.m10, matrix.m11, matrix.m12,
            matrix.m20, matrix.m21, matrix.m22
        );
    }

    /**
     * Non-standard matrix naming (xy means xVector's yComponent)
     * Matrix must be positive unit orthogonal
     * @return A new quaternion representing the same rotation as the matrix
     */
    public static Quaternion fromRotationMatrix(
        float xx, float yx, float zx,
        float xy, float yy, float zy,
        float xz, float yz, float zz
    ) {
        Quaternion output = new Quaternion();
        return output.fromRotationMatrixLocal(
            xx, yx, zx,
            xy, yy, zy,
            xz, yz, zz
        );
    }

    /**
     * Matrix must be positive unit orthogonal
     * @return A new quaternion representing the same rotation as the matrix
     */
    public static Quaternion fromRotationMatrix(Matrix3f matrix) {
        Quaternion output = new Quaternion();
        return output.fromRotationMatrixLocal(
            matrix.m00, matrix.m01, matrix.m02,
            matrix.m10, matrix.m11, matrix.m12,
            matrix.m20, matrix.m21, matrix.m22
        );
    }




    /**
     * Sets this to represent a rotataion by the rotation vector
     * Rotation vector describes a rotation about its direction with an angle of its magnitude
     * @return this
     */
    public Quaternion fromRotationVectorLocal(float rx, float ry, float rz) {
        float len = (float) Math.sqrt(rx*rx + ry*ry + rz*rz);
        if (len == 0f) {
            return new Quaternion();
        }

        float cos = (float) Math.cos(0.5f*len);
        float sin = (float) Math.sin(0.5f*len);
        float inv = 1f/len;
        
        this.w = cos;
        this.x = sin*inv*rx;
        this.y = sin*inv*ry;
        this.z = sin*inv*rz;

        return this;
    }

    /**
     * Sets this to represent a rotataion by the rotation vector
     * Rotation vector describes a rotation about its direction with an angle of its magnitude
     * @return this
     */
    public Quaternion fromRotationVectorLocal(Vector3f rotationVector) {
        return this.fromRotationVectorLocal(
            rotationVector.x,
            rotationVector.y,
            rotationVector.z
        );
    }

    /**
     * Rotation vector describes a rotation about its direction with an angle of its magnitude
     * @return A new quaternion representing a rotataion by the rotation vector
     */
    public static Quaternion fromRotationVector(float rx, float ry, float rz) {
        Quaternion output = new Quaternion();
        return output.fromRotationVectorLocal(rx, ry, rz);
    }

    /**
     * Rotation vector describes a rotation about its direction with an angle of its magnitude
     * @return A new quaternion representing a rotataion by the rotation vector
     */
    public static Quaternion fromRotationVector(Vector3f rotationVector) {
        Quaternion output = new Quaternion();
        return output.fromRotationVectorLocal(
            rotationVector.x,
            rotationVector.y,
            rotationVector.z
        );
    }




    /**
     * Sets this to represent a rotation about an axis by an angle
     * @return this
     */
    public Quaternion fromAngleAxisLocal(float ang, float ax, float ay, float az) {
        float len = (float) Math.sqrt(ax*ax + ay*ay + az*az);
        if (len == 0f) {
            w = 1f; // technically not defined but sure
            x = 0f;
            y = 0f;
            z = 0f;
        }

        float cos = (float) Math.cos(0.5f*ang);
        float sin = (float) Math.sin(0.5f*ang);
        float inv = 1f/len;

        this.w = cos;
        this.x = sin*inv*ax;
        this.y = sin*inv*ay;
        this.z = sin*inv*az;

        return this;
    }

    /**
     * Sets this to represent a rotation about an axis by an angle
     * @return this
     */
    public Quaternion fromAngleAxisLocal(float ang, Vector3f axis) {
        return this.fromAngleAxisLocal(ang, axis.x, axis.y, axis.z);
    }

    /**
     * @return A new quaternion representing a rotation about an axis by an angle
     */
    public static Quaternion fromAngleAxis(float ang, float ax, float ay, float az) {
        Quaternion output = new Quaternion();
        return output.fromAngleAxisLocal(ang, ax, ay, az);
    }

    /**
     * @return A new quaternion representing a rotation about an axis by an angle
     */
    public static Quaternion fromAngleAxis(float ang, Vector3f axis) {
        Quaternion output = new Quaternion();
        return output.fromAngleAxisLocal(ang, axis.x, axis.y, axis.z);
    }




    /**
     * Sets this to represent the result of rotations about the cardinal axes in the order X Y Z
     * @param X The angle by which to rotate about the X axis
     * @param Y The angle by which to rotate about the Y axis
     * @param Z The angle by which to rotate about the Z axis
     * @return this
     */
    public Quaternion fromEulerXYZLocal(float X, float Y, float Z) {
        float cosX = (float) Math.cos(0.5f*X);
        float cosY = (float) Math.cos(0.5f*Y);
        float cosZ = (float) Math.cos(0.5f*Z);
        float sinX = (float) Math.sin(0.5f*X);
        float sinY = (float) Math.sin(0.5f*Y);
        float sinZ = (float) Math.sin(0.5f*Z);

        this.w = cosX*cosY*cosZ - sinX*sinY*sinZ;
        this.x = cosY*cosZ*sinX + cosX*sinY*sinZ;
        this.y = cosX*cosZ*sinY - cosY*sinX*sinZ;
        this.z = cosZ*sinX*sinY + cosX*cosY*sinZ;

        return this;
    }

    /**
     * Sets this to represent the result of rotations about the cardinal axes in the order X Z Y
     * @param X The angle by which to rotate about the X axis
     * @param Z The angle by which to rotate about the Z axis
     * @param Y The angle by which to rotate about the Y axis
     * @return this
     */
    public Quaternion fromEulerXZYLocal(float X, float Z, float Y) {
        float cosX = (float) Math.cos(0.5f*X);
        float cosY = (float) Math.cos(0.5f*Y);
        float cosZ = (float) Math.cos(0.5f*Z);
        float sinX = (float) Math.sin(0.5f*X);
        float sinY = (float) Math.sin(0.5f*Y);
        float sinZ = (float) Math.sin(0.5f*Z);

        this.w = cosX*cosY*cosZ + sinX*sinY*sinZ;
        this.x = cosY*cosZ*sinX - cosX*sinY*sinZ;
        this.y = cosX*cosZ*sinY - cosY*sinX*sinZ;
        this.z = cosZ*sinX*sinY + cosX*cosY*sinZ;

        return this;
    }

    /**
     * Sets this to represent the result of rotations about the cardinal axes in the order Y X Z
     * @param Y The angle by which to rotate about the Y axis
     * @param X The angle by which to rotate about the X axis
     * @param Z The angle by which to rotate about the Z axis
     * @return this
     */
    public Quaternion fromEulerYXZLocal(float Y, float X, float Z) {
        float cosX = (float) Math.cos(0.5f*X);
        float cosY = (float) Math.cos(0.5f*Y);
        float cosZ = (float) Math.cos(0.5f*Z);
        float sinX = (float) Math.sin(0.5f*X);
        float sinY = (float) Math.sin(0.5f*Y);
        float sinZ = (float) Math.sin(0.5f*Z);

        this.w = cosX*cosY*cosZ + sinX*sinY*sinZ;
        this.x = cosY*cosZ*sinX + cosX*sinY*sinZ;
        this.y = cosX*cosZ*sinY - cosY*sinX*sinZ;
        this.z = cosX*cosY*sinZ - cosZ*sinX*sinY;

        return this;
    }

    /**
     * Sets this to represent the result of rotations about the cardinal axes in the order Y Z X
     * @param Y The angle by which to rotate about the Y axis
     * @param Z The angle by which to rotate about the Z axis
     * @param X The angle by which to rotate about the X axis
     * @return this
     */
    public Quaternion fromEulerYZXLocal(float Y, float Z, float X) {
        float cosX = (float) Math.cos(0.5f*X);
        float cosY = (float) Math.cos(0.5f*Y);
        float cosZ = (float) Math.cos(0.5f*Z);
        float sinX = (float) Math.sin(0.5f*X);
        float sinY = (float) Math.sin(0.5f*Y);
        float sinZ = (float) Math.sin(0.5f*Z);

        this.w = cosX*cosY*cosZ - sinX*sinY*sinZ;
        this.x = cosY*cosZ*sinX + cosX*sinY*sinZ;
        this.y = cosX*cosZ*sinY + cosY*sinX*sinZ;
        this.z = cosX*cosY*sinZ - cosZ*sinX*sinY;

        return this;
    }

    /**
     * Sets this to represent the result of rotations about the cardinal axes in the order Z X Y
     * @param Z The angle by which to rotate about the Z axis
     * @param X The angle by which to rotate about the X axis
     * @param Y The angle by which to rotate about the Y axis
     * @return this
     */
    public Quaternion fromEulerZXYLocal(float Z, float X, float Y) {
        float cosX = (float) Math.cos(0.5f*X);
        float cosY = (float) Math.cos(0.5f*Y);
        float cosZ = (float) Math.cos(0.5f*Z);
        float sinX = (float) Math.sin(0.5f*X);
        float sinY = (float) Math.sin(0.5f*Y);
        float sinZ = (float) Math.sin(0.5f*Z);

        this.w = cosX*cosY*cosZ - sinX*sinY*sinZ;
        this.x = cosY*cosZ*sinX - cosX*sinY*sinZ;
        this.y = cosX*cosZ*sinY + cosY*sinX*sinZ;
        this.z = cosZ*sinX*sinY + cosX*cosY*sinZ;

        return this;
    }

    /**
     * Sets this to represent the result of rotations about the cardinal axes in the order Z Y X
     * @param Z The angle by which to rotate about the Z axis
     * @param Y The angle by which to rotate about the Y axis
     * @param X The angle by which to rotate about the X axis
     * @return this
     */
    public Quaternion fromEulerZYXLocal(float Z, float Y, float X) {
        float cosX = (float) Math.cos(0.5f*X);
        float cosY = (float) Math.cos(0.5f*Y);
        float cosZ = (float) Math.cos(0.5f*Z);
        float sinX = (float) Math.sin(0.5f*X);
        float sinY = (float) Math.sin(0.5f*Y);
        float sinZ = (float) Math.sin(0.5f*Z);

        this.w = cosX*cosY*cosZ + sinX*sinY*sinZ;
        this.x = cosY*cosZ*sinX - cosX*sinY*sinZ;
        this.y = cosX*cosZ*sinY + cosY*sinX*sinZ;
        this.z = cosX*cosY*sinZ - cosZ*sinX*sinY;

        return this;
    }



    /**
     * Sets this to represent the result of rotations about the cardinal axes in the order X Y Z
     * @param angles The angles in the order X Y Z
     * @return this
     */
    public Quaternion fromEulerXYZLocal(float[] angles) {
        return this.fromEulerXYZLocal(angles[0], angles[1], angles[2]);
    }

    /**
     * Sets this to represent the result of rotations about the cardinal axes in the order X Y Z
     * @param angles The angles in the order X Z Y
     * @return this
     */
    public Quaternion fromEulerXZYLocal(float[] angles) {
        return this.fromEulerXZYLocal(angles[0], angles[1], angles[2]);
    }

    /**
     * Sets this to represent the result of rotations about the cardinal axes in the order X Y Z
     * @param angles The angles in the order Y X Z
     * @return this
     */
    public Quaternion fromEulerYXZLocal(float[] angles) {
        return this.fromEulerYXZLocal(angles[0], angles[1], angles[2]);
    }

    /**
     * Sets this to represent the result of rotations about the cardinal axes in the order X Y Z
     * @param angles The angles in the order Y Z X
     * @return this
     */
    public Quaternion fromEulerYZXLocal(float[] angles) {
        return this.fromEulerYZXLocal(angles[0], angles[1], angles[2]);
    }

    /**
     * Sets this to represent the result of rotations about the cardinal axes in the order X Y Z
     * @param angles The angles in the order Z X Y
     * @return this
     */
    public Quaternion fromEulerZXYLocal(float[] angles) {
        return this.fromEulerZXYLocal(angles[0], angles[1], angles[2]);
    }

    /**
     * Sets this to represent the result of rotations about the cardinal axes in the order X Y Z
     * @param angles The angles in the order Z Y X
     * @return this
     */
    public Quaternion fromEulerZYXLocal(float[] angles) {
        return this.fromEulerZYXLocal(angles[0], angles[1], angles[2]);
    }




    /**
     * @param X The angle by which to rotate about the X axis
     * @param Y The angle by which to rotate about the Y axis
     * @param Z The angle by which to rotate about the Z axis
     * @return A new quaternion representing the result of rotations about the cardinal axes in the order X Y Z
     */
    public static Quaternion fromEulerXYZ(float X, float Y, float Z) {
        Quaternion output = new Quaternion();
        return output.fromEulerXYZLocal(X, Y, Z);
    }

    /**
     * @param X The angle by which to rotate about the X axis
     * @param Z The angle by which to rotate about the Z axis
     * @param Y The angle by which to rotate about the Y axis
     * @return A new quaternion representing the result of rotations about the cardinal axes in the order X Z Y
     */
    public static Quaternion fromEulerXZY(float X, float Z, float Y) {
        Quaternion output = new Quaternion();
        return output.fromEulerXZYLocal(X, Z, Y);
    }

    /**
     * @param Y The angle by which to rotate about the Y axis
     * @param X The angle by which to rotate about the X axis
     * @param Z The angle by which to rotate about the Z axis
     * @return A new quaternion representing the result of rotations about the cardinal axes in the order Y X Z
     */
    public static Quaternion fromEulerYXZ(float Y, float X, float Z) {
        Quaternion output = new Quaternion();
        return output.fromEulerYXZLocal(Y, X, Z);
    }

    /**
     * @param Y The angle by which to rotate about the Y axis
     * @param Z The angle by which to rotate about the Z axis
     * @param X The angle by which to rotate about the X axis
     * @return A new quaternion representing the result of rotations about the cardinal axes in the order Y Z X
     */
    public static Quaternion fromEulerYZX(float Y, float Z, float X) {
        Quaternion output = new Quaternion();
        return output.fromEulerYZXLocal(Y, Z, X);
    }

    /**
     * @param Z The angle by which to rotate about the Z axis
     * @param X The angle by which to rotate about the X axis
     * @param Y The angle by which to rotate about the Y axis
     * @return A new quaternion representing the result of rotations about the cardinal axes in the order Z X Y
     */
    public static Quaternion fromEulerZXY(float Z, float X, float Y) {
        Quaternion output = new Quaternion();
        return output.fromEulerZXYLocal(Z, X, Y);
    }

    /**
     * @param Z The angle by which to rotate about the Z axis
     * @param Y The angle by which to rotate about the Y axis
     * @param X The angle by which to rotate about the X axis
     * @return A new quaternion representing the result of rotations about the cardinal axes in the order Z Y X
     */
    public static Quaternion fromEulerZYX(float Z, float Y, float X) {
        Quaternion output = new Quaternion();
        return output.fromEulerZYXLocal(Z, Y, X);
    }




    /**
     * @param angles An array containing angles in the order X Y Z
     * @return A new quaternion representing the result of rotations about the cardinal axes in the order X Y Z
     */
    public static Quaternion fromEulerXYZ(float[] angles) {
        Quaternion output = new Quaternion();
        return output.fromEulerXYZLocal(angles[0], angles[1], angles[2]);
    }

    /**
     * @param angles An array containing angles in the order X Z Y
     * @return A new quaternion representing the result of rotations about the cardinal axes in the order X Z Y
     */
    public static Quaternion fromEulerXZY(float[] angles) {
        Quaternion output = new Quaternion();
        return output.fromEulerXZYLocal(angles[0], angles[1], angles[2]);
    }

    /**
     * @param angles An array containing angles in the order Y X Z
     * @return A new quaternion representing the result of rotations about the cardinal axes in the order Y X Z
     */
    public static Quaternion fromEulerYXZ(float[] angles) {
        Quaternion output = new Quaternion();
        return output.fromEulerYXZLocal(angles[0], angles[1], angles[2]);
    }

    /**
     * @param angles An array containing angles in the order Y Z X
     * @return A new quaternion representing the result of rotations about the cardinal axes in the order Y Z X
     */
    public static Quaternion fromEulerYZX(float[] angles) {
        Quaternion output = new Quaternion();
        return output.fromEulerYZXLocal(angles[0], angles[1], angles[2]);
    }

    /**
     * @param angles An array containing angles in the order Z X Y
     * @return A new quaternion representing the result of rotations about the cardinal axes in the order Z X Y
     */
    public static Quaternion fromEulerZXY(float[] angles) {
        Quaternion output = new Quaternion();
        return output.fromEulerZXYLocal(angles[0], angles[1], angles[2]);
    }

    /**
     * @param angles An array containing angles in the order Z Y X
     * @return A new quaternion representing the result of rotations about the cardinal axes in the order Z Y X
     */
    public static Quaternion fromEulerZYX(float[] angles) {
        Quaternion output = new Quaternion();
        return output.fromEulerZYXLocal(angles[0], angles[1], angles[2]);
    }




    /**
     * Sets output to containing euler angles X Y Z in the order X Y Z representing the same rotation as this quaternion
     * @return output
     */
    public float[] toEulerXYZ(float[] output) {
        float zz = w*w - x*x - y*y + z*z;
        float zy = 2*(y*z - w*x);
        float kc = (float) Math.sqrt(zy*zy + zz*zz);
        float zx = 2*(w*y + x*z);
        float xx = w*w + x*x - y*y - z*z;
        float yx = 2*(x*y - w*z);

        float X, Y, Z;
        if ((zx < 0 ? -zx : zx) > EULER_TOL*kc) {
            X = 2f*(float) Math.atan2(x, w);
            Y = (float) Math.atan2( zx, kc);
            Z = 0f;
        } else {
            X = (float) Math.atan2(-zy, zz);
            Y = (float) Math.atan2( zx, kc);
            Z = (float) Math.atan2(-yx, xx);
        }

        output[0] = X;
        output[1] = Y;
        output[2] = Z;

        return output;
    }

    /**
     * Sets output to containing euler angles Y Z X in the order Y Z X representing the same rotation as this quaternion
     * @return output
     */
    public float[] toEulerYZX(float[] output) {
        float xx = w*w + x*x - y*y - z*z;
        float xz = 2*(x*z - w*y);
        float kc = (float) Math.sqrt(xz*xz + xx*xx);
        float xy = 2*(x*y + w*z);
        float yy = w*w - x*x + y*y - z*z;
        float zy = 2*(y*z - w*x);

        float Y, Z, X;
        if ((xy < 0 ? -xy : xy) > EULER_TOL*kc) {
            Y = 2f*(float) Math.atan2(y, w);
            Z = (float) Math.atan2( xy, kc);
            X = 0f;
        } else {
            Y = (float) Math.atan2(-xz, xx);
            Z = (float) Math.atan2( xy, kc);
            X = (float) Math.atan2(-zy, yy);
        }

        output[0] = Y;
        output[1] = Z;
        output[2] = X;

        return output;
    }

    /**
     * Sets output to containing euler angles Z X Y in the order Z X Y representing the same rotation as this quaternion
     * @return output
     */
    public float[] toEulerZXY(float[] output) {
        float yy = w*w - x*x + y*y - z*z;
        float yx = 2*(x*y - w*z);
        float kc = (float) Math.sqrt(yx*yx + yy*yy);
        float yz = 2*(w*x + y*z);
        float zz = w*w - x*x - y*y + z*z;
        float xz = 2*(x*z - w*y);

        float Z, X, Y;
        if ((yz < 0 ? -yz : yz) > EULER_TOL*kc) {
            Z = 2f*(float) Math.atan2(z, w);
            X = (float) Math.atan2( yz, kc);
            Y = 0f;
        } else {
            Z = (float) Math.atan2(-yx, yy);
            X = (float) Math.atan2( yz, kc);
            Y = (float) Math.atan2(-xz, zz);
        }

        output[0] = Z;
        output[1] = X;
        output[2] = Y;

        return output;
    }



    /**
     * Sets output to containing euler angles Z Y X in the order Z Y X representing the same rotation as this quaternion
     * @return output
     */
    public float[] toEulerZYX(float[] output) {
        float xx = w*w + x*x - y*y - z*z;
        float xy = 2*(x*y + w*z);
        float kc = (float) Math.sqrt(xy*xy + xx*xx);
        float xz = 2*(x*z - w*y);
        float zz = w*w - x*x - y*y + z*z;
        float yz = 2*(w*x + y*z);

        float Z, Y, X;
        if ((xz < 0 ? -xz : xz) > EULER_TOL*kc) {
            Z = 2f*(float) Math.atan2(z, w);
            Y = (float) Math.atan2(-xz, kc);
            X = 0f;
        } else {
            Z = (float) Math.atan2( xy, xx);
            Y = (float) Math.atan2(-xz, kc);
            X = (float) Math.atan2( yz, zz);
        }

        output[0] = Z;
        output[1] = Y;
        output[2] = X;

        return output;
    }

    /**
     * Sets output to containing euler angles Y X Z in the order Y X Z representing the same rotation as this quaternion
     * @return output
     */
    public float[] toEulerYXZ(float[] output) {
        float zz = w*w - x*x - y*y + z*z;
        float zx = 2*(w*y + x*z);
        float kc = (float) Math.sqrt(zx*zx + zz*zz);
        float zy = 2*(y*z - w*x);
        float yy = w*w - x*x + y*y - z*z;
        float xy = 2*(x*y + w*z);

        float Y, X, Z;
        if ((zy < 0 ? -zy : zy) > EULER_TOL*kc) {
            Y = 2f*(float) Math.atan2(y, w);
            X = (float) Math.atan2(-zy, kc);
            Z = 0f;
        } else {
            Y = (float) Math.atan2( zx, zz);
            X = (float) Math.atan2(-zy, kc);
            Z = (float) Math.atan2( xy, yy);
        }

        output[0] = Y;
        output[1] = X;
        output[2] = Z;

        return output;
    }

    /**
     * Sets output to containing euler angles X Z Y in the order X Z Y representing the same rotation as this quaternion
     * @return output
     */
    public float[] toEulerXZY(float[] output) {
        float yy = w*w - x*x + y*y - z*z;
        float yz = 2*(w*x + y*z);
        float kc = (float) Math.sqrt(yz*yz + yy*yy);
        float yx = 2*(x*y - w*z);
        float xx = w*w + x*x - y*y - z*z;
        float zx = 2*(w*y + x*z);

        float X, Z, Y;
        if ((yx < 0 ? -yx : yx) > EULER_TOL*kc) {
            X = 2f*(float) Math.atan2(x, w);
            Z = (float) Math.atan2(-yx, kc);
            Y = 0f;
        } else {
            X = (float) Math.atan2( yz, yy);
            Z = (float) Math.atan2(-yx, kc);
            Y = (float) Math.atan2( zx, xx);
        }

        output[0] = X;
        output[1] = Z;
        output[2] = Y;

        return output;
    }




    /**
     * @return A 3-array containing euler angles X Y Z in the order X Y Z representing the same rotation as this quaternion
     */
    public float[] toEulerXYZ() {
        float[] output = new float[3];
        return this.toEulerXYZ(output);
    }

    /**
     * @return A 3-array containing euler angles X Z Y in the order X Z Y representing the same rotation as this quaternion
     */
    public float[] toEulerXZY() {
        float[] output = new float[3];
        return this.toEulerXZY(output);
    }

    /**
     * @return A 3-array containing euler angles Y X Z in the order Y X Z representing the same rotation as this quaternion
     */
    public float[] toEulerYXZ() {
        float[] output = new float[3];
        return this.toEulerYXZ(output);
    }

    /**
     * @return A 3-array containing euler angles Y Z X in the order Y Z X representing the same rotation as this quaternion
     */
    public float[] toEulerYZX() {
        float[] output = new float[3];
        return this.toEulerYZX(output);
    }

    /**
     * @return A 3-array containing euler angles Z X Y in the order Z X Y representing the same rotation as this quaternion
     */
    public float[] toEulerZXY() {
        float[] output = new float[3];
        return this.toEulerZXY(output);
    }

    /**
     * @return A 3-array containing euler angles Z Y X in the order Z Y X representing the same rotation as this quaternion
     */
    public float[] toEulerZYX() {
        float[] output = new float[3];
        return this.toEulerZYX(output);
    }




    // Please use these methods
    // Garbage collection will not hate you, I promise.

    public Quaternion unit() {
        Quaternion output = new Quaternion();
        return output.unit(this);
    }

    public Quaternion neg() {
        Quaternion output = new Quaternion();
        return output.neg(this);
    }

    public Quaternion conj() {
        Quaternion output = new Quaternion();
        return output.conj(this);
    }

    public Quaternion add(Quaternion that) {
        Quaternion output = new Quaternion();
        return output.add(this, that);
    }

    public Quaternion sub(Quaternion that) {
        Quaternion output = new Quaternion();
        return output.sub(this, that);
    }

    public Quaternion mul(Quaternion that) {
        Quaternion output = new Quaternion();
        return output.mul(this, that);
    }

    public Quaternion mul(float that) {
        Quaternion output = new Quaternion();
        return output.mul(this, that);
    }

    public Quaternion div(float that) {
        Quaternion output = new Quaternion();
        return output.mul(this, that);
    }

    public Quaternion inv() {
        Quaternion output = new Quaternion();
        return output.inv(this);
    }

    public Quaternion invMul(Quaternion that) {
        Quaternion output = new Quaternion();
        return output.invMul(this, that);
    }

    public Quaternion mulInv(Quaternion that) {
        Quaternion output = new Quaternion();
        return output.mulInv(this, that);
    }


    // Transformers
    public Matrix3f toRotationMatrix3(Matrix3f output) {
        float inv = 1f/(w*w + x*x + y*y + z*z);

        output.m00 = inv*(w*w + x*x - y*y - z*z);
        output.m01 = inv*2f*(x*y - w*z);
        output.m02 = inv*2f*(w*y + x*z);
        output.m10 = inv*2f*(x*y + w*z);
        output.m11 = inv*(w*w - x*x + y*y - z*z);
        output.m12 = inv*2f*(y*z - w*x);
        output.m20 = inv*2f*(x*z - w*y);
        output.m21 = inv*2f*(w*x + y*z);
        output.m22 = inv*(w*w - x*x - y*y + z*z);

        return output;
    }
    
    public Matrix3f toRotationMatrix3() {
        Matrix3f output = new Matrix3f();
        return this.toRotationMatrix3(output);
    }

    public Vector3f toRotationVector(Vector3f output) {
        float im = (float) Math.sqrt(x*x + y*y + z*z);
        if (im == 0) {
            output.x = 0;
            output.y = 0;
            output.z = 0;

            return output;
        }

        float mul = 2f*(float) Math.atan2(im, w)/im;

        output.x = mul*x;
        output.y = mul*y;
        output.z = mul*z;

        return output;
    }

    public Vector3f toAxis(Vector3f output) {
        float im = (float) Math.sqrt(x*x + y*y + z*z);
        if (im == 0) {
            output.x = 1; // arbitrary
            output.y = 0;
            output.z = 0;

            return output;
        }

        float mul = 1f/im;
        output.x = mul*x;
        output.y = mul*y;
        output.z = mul*z;

        return output;
    }

    public Vector3f toAxis() {
        Vector3f output = new Vector3f();
        return this.toAxis(output);
    }

    public float toAngle() {
        float im = (float) Math.sqrt(x*x + y*y + z*z);
        float ang = 2f*(float) Math.atan2(im, w);

        return ang;
    }

    public float toAngleAxis(Vector3f output) {
        float im = (float) Math.sqrt(x*x + y*y + z*z);

        if (im == 0) {
            output.x = 1; // arbitrary
            output.y = 0;
            output.z = 0;

            return 0;
        }

        float mul = 1f/im;
        output.x = mul*x;
        output.y = mul*y;
        output.z = mul*z;

        float ang = 2f*(float) Math.atan2(im, w);

        return ang;
    }



    // Rotates the vector by the rotation described by this quaternion
    public Vector3f sandwich(float vx, float vy, float vz, Vector3f output) {
        float inv = 1f/(w*w + x*x + y*y + z*z);

        //b = v*inverse(this)
        float bw = inv*(vx*x + vy*y + vz*z);
        float bx = inv*(vx*w + vz*y - vy*z);
        float by = inv*(vy*w - vz*x + vx*z);
        float bz = inv*(vz*w + vy*x - vx*y);

        // output = this*v*inverse(this)
        output.x = w*bx + x*bw + y*bz - z*by;
        output.y = w*by - x*bz + y*bw + z*bx;
        output.z = w*bz + x*by - y*bx + z*bw;

        return output;
    }

    public Vector3f sandwich(Vector3f vector, Vector3f output) {
        return this.sandwich(vector.x, vector.y, vector.z, output);
    }

    public Vector3f sandwich(float vx, float vy, float vz) {
        Vector3f output = new Vector3f();
        return this.sandwich(vx, vy, vz, output);
    }

    public Vector3f sandwich(Vector3f vector) {
        Vector3f output = new Vector3f();
        return this.sandwich(vector.x, vector.y, vector.z, output);
    }




    public float norm() {
        return w*w + x*x + y*y + z*z;
    }

    public float dot(Quaternion that) {
        return w*that.w + x*that.x + y*that.y + z*that.z;
    }

    public float mag() {
        return (float) Math.sqrt(w*w + x*x + y*y + z*z);
    }

    public Quaternion unit(Quaternion A) {
        float inv = 1f/(float) Math.sqrt(A.w*A.w + A.x*A.x + A.y*A.y + A.z*A.z);
        w = inv*A.w;
        x = inv*A.x;
        y = inv*A.y;
        z = inv*A.z;

        return this;
    }

    // -A -> this
    public Quaternion neg(Quaternion A) {
        w = -A.w;
        x = -A.x;
        y = -A.y;
        z = -A.z;

        return this;
    }
    
    public Quaternion conj(Quaternion A) {
        w = A.w;
        x = -A.x;
        y = -A.y;
        z = -A.z;

        return this;
    }

    // A + B -> this
    public Quaternion add(Quaternion A, Quaternion B) {
        w = A.w + B.w;
        x = A.x + B.x;
        y = A.y + B.y;
        z = A.z + B.z;

        return this;
    }

    // A - B -> this
    public Quaternion sub(Quaternion A, Quaternion B) {
        w = A.w - B.w;
        x = A.x - B.x;
        y = A.y - B.y;
        z = A.z - B.z;

        return this;
    }

    // A * B -> this
    public Quaternion mul(Quaternion A, Quaternion B) {
        float Cw = A.w*B.w - A.x*B.x - A.y*B.y - A.z*B.z;
        float Cx = A.x*B.w + A.w*B.x - A.z*B.y + A.y*B.z;
        float Cy = A.y*B.w + A.z*B.x + A.w*B.y - A.x*B.z;
        float Cz = A.z*B.w - A.y*B.x + A.x*B.y + A.w*B.z;

        w = Cw;
        x = Cx;
        y = Cy;
        z = Cz;

        return this;
    }

    // A * b -> this
    public Quaternion mul(Quaternion A, float b) {
        this.w = A.w*b;
        this.x = A.x*b;
        this.y = A.y*b;
        this.z = A.z*b;

        return this;
    }

    // A / b -> this
    public Quaternion div(Quaternion A, float b) {
        this.w = A.w/b;
        this.x = A.x/b;
        this.y = A.y/b;
        this.z = A.z/b;

        return this;
    }

    // A^-1 -> this
    public Quaternion inv(Quaternion A) {
        float inv = 1f/(A.w*A.w + A.x*A.x + A.y*A.y + A.z*A.z);
        w = inv*A.w;
        x = inv*-A.x;
        y = inv*-A.y;
        z = inv*-A.z;

        return this;
    }

    // A^-1 * B -> this
    public Quaternion invMul(Quaternion A, Quaternion B) {
        float inv = 1f/(A.w*A.w + A.x*A.x + A.y*A.y + A.z*A.z);
        float Cw = inv*(A.w*B.w + A.x*B.x + A.y*B.y + A.z*B.z);
        float Cx = inv*(A.w*B.x - A.x*B.w - A.y*B.z + A.z*B.y);
        float Cy = inv*(A.w*B.y + A.x*B.z - A.y*B.w - A.z*B.x);
        float Cz = inv*(A.w*B.z - A.x*B.y + A.y*B.x - A.z*B.w);

        w = Cw;
        x = Cx;
        y = Cy;
        z = Cz;

        return this;
    }

    // A * B^-1 -> this
    public Quaternion mulInv(Quaternion A, Quaternion B) {
        float inv = 1f/(B.w*B.w + B.x*B.x + B.y*B.y + B.z*B.z);
        float Cw = inv*(A.w*B.w + A.x*B.x + A.y*B.y + A.z*B.z);
        float Cx = inv*(A.x*B.w - A.w*B.x + A.z*B.y - A.y*B.z);
        float Cy = inv*(A.y*B.w - A.z*B.x - A.w*B.y + A.x*B.z);
        float Cz = inv*(A.z*B.w + A.y*B.x - A.x*B.y - A.w*B.z);

        w = Cw;
        x = Cx;
        y = Cy;
        z = Cz;

        return this;
    }




    // QUATERNION PROJECTION

    public Quaternion project(Quaternion Q, float ax, float ay, float az) {
        float aNorm = ax*ax + ay*ay + az*az;
        float aDotQ = Q.x*ax + Q.y*ay + Q.z*az;

        float t = aDotQ/aNorm;
        this.w = Q.w;
        this.x = t*ax;
        this.y = t*ay;
        this.z = t*az;

        return this;
    }

    public Quaternion project(Quaternion Q, Vector3f axis) {
        return this.project(Q, axis.x, axis.y, axis.z);
    }

    public Quaternion project(float ax, float ay, float az) {
        Quaternion output = new Quaternion();
        return output.project(this, ax, ay, az);
    }

    public Quaternion project(Vector3f axis) {
        Quaternion output = new Quaternion();
        return output.project(this, axis.x, axis.y, axis.z);
    }




    public Quaternion projectUnitize(Quaternion Q, float ax, float ay, float az) {
        return this.project(Q, ax, ay, az).unit(this);
    }

    public Quaternion projectUnitize(Quaternion Q, Vector3f axis) {
        return this.projectUnitize(Q, axis.x, axis.y, axis.z);
    }

    public Quaternion projectUnitize(float ax, float ay, float az) {
        Quaternion output = new Quaternion();
        return output.projectUnitize(this, ax, ay, az);
    }

    public Quaternion projectUnitize(Vector3f axis) {
        Quaternion output = new Quaternion();
        return output.projectUnitize(this, axis.x, axis.y, axis.z);
    }




    public float projectedAngle(float ax, float ay, float az) {
        float aMag = (float) Math.sqrt(ax*ax + ay*ay + az*az);
        float aDotQ = this.x*ax + this.y*ay + this.z*az;

        float ang = (float) Math.atan2(aDotQ/aMag, this.w);
        return ang;
    }

    public float projectedAngle(Vector3f axis) {
        return this.projectedAngle(axis.x, axis.y, axis.z);
    }



    // TODO: Implement alignUnitize //
    // Implement projectUnitize //
    // handle undefined cases of toEuler___ //
    // Implement mulMulInv A*B*C^-1 maybe
    // Make methods for fromAngleAxis //
    // Implement getAngle //
    // implement getAxis //
    // implement getAngleAxis //
    // implement sandwich(vx, vy, vz) //
    // estimated time: 3 hours

    // Test it all
    // estimated time: 20 hours

    // QUATERNION ALIGNMENT

    // b*Q*a^-1 + mag(b*a^-1)*Q
    // -b*Q*a/a.a + mag(b.b/a.a)*Q
    // Applies a minimal rotation R to Q such that R*Q*a = b
    // Highly useful, deserves a place in most quaternion libraries
    // Warning if Q*a = -b, then no R can be found and results in division by 0
    public Quaternion align(
        Quaternion Q,
        float ax, float ay, float az,
        float bx, float by, float bz
    ) {
        float aNorm = ax*ax + ay*ay + az*az;
        float bNorm = bx*bx + by*by + bz*bz;

        float aNormInv = 1f/aNorm;

        // R = Q*a^-1, a is pure imaginary quaternion
        float Rw = aNormInv*(Q.x*ax + Q.y*ay + Q.z*az);
        float Rx = aNormInv*(Q.z*ay - Q.w*ax - Q.y*az);
        float Ry = aNormInv*(Q.x*az - Q.w*ay - Q.z*ax);
        float Rz = aNormInv*(Q.y*ax - Q.w*az - Q.x*ay);

        // S = b*Q*a^-1, a and b are pure imaginary quaternions
        float Sw = -bx*Rx - by*Ry - bz*Rz;
        float Sx =  bx*Rw - bz*Ry + by*Rz;
        float Sy =  by*Rw + bz*Rx - bx*Rz;
        float Sz =  bz*Rw - by*Rx + bx*Ry;

        float mul = (float) Math.sqrt(aNormInv*bNorm);

        this.w = Sw + mul*Q.w;
        this.x = Sx + mul*Q.x;
        this.y = Sy + mul*Q.y;
        this.z = Sz + mul*Q.z;

        return this;
    }

    public Quaternion align(Quaternion Q, Vector3f a, Vector3f b) {
        return this.align(Q, a.x, a.y, a.z, b.x, b.y, b.z);
    }

    public Quaternion align(
        float ax, float ay, float az,
        float bx, float by, float bz
    ) {
        Quaternion output = new Quaternion();
        return output.align(this, ax, ay, az, bx, by, bz);
    }

    public Quaternion align(Vector3f a, Vector3f b) {
        Quaternion output = new Quaternion();
        return output.align(this, a.x, a.y, a.z, b.x, b.y, b.z);
    }




    public Quaternion alignUnitize(
        Quaternion Q, 
        float ax, float ay, float az,
        float bx, float by, float bz
    ) {
        return this.align(Q, ax, ay, az, bx, by, bz).unit(this);
    }

    public Quaternion alignUnitize(Quaternion Q, Vector3f a, Vector3f b) {
        return this.alignUnitize(Q, a.x, a.y, a.z, b.x, b.y, b.z);
    }

    public Quaternion alignUnitize(
        float ax, float ay, float az,
        float bx, float by, float bz
    ) {
        Quaternion output = new Quaternion();
        return output.alignUnitize(this, ax, ay, az, bx, by, bz);
    }

    public Quaternion alignUnitize(Vector3f a, Vector3f b) {
        Quaternion output = new Quaternion();
        return output.alignUnitize(this, a.x, a.y, a.z, b.x, b.y, b.z);
    }




    // ANGLE BETWEEN

    public float angleBetween(Quaternion that) {
        float Rw = this.w*that.w + this.x*that.x + this.y*that.y + this.z*that.z;
        float Rx = this.w*that.x - this.x*that.w - this.y*that.z + this.z*that.y;
        float Ry = this.w*that.y + this.x*that.z - this.y*that.w - this.z*that.x;
        float Rz = this.w*that.z - this.x*that.y + this.y*that.x - this.z*that.w;

        // compute cosine and sine of the angle between
        // do so in a numerically stable way
        return (float) Math.atan2(Math.sqrt(Rx*Rx + Ry*Ry + Rz*Rz), (double) Rw);
    }




    // SLERP DEFINITIONS

    public Quaternion slerp(
        float Aw, float Ax, float Ay, float Az,
        float Bw, float Bx, float By, float Bz,
        float t
    ) {
        // get B relative to A
        float Rw = Aw*Bw + Ax*Bx + Ay*By + Az*Bz;
        float Rx = Aw*Bx - Ax*Bw - Ay*Bz + Az*By;
        float Ry = Aw*By + Ax*Bz - Ay*Bw - Az*Bx;
        float Rz = Aw*Bz - Ax*By + Ay*Bx - Az*Bw;

        // compute theta robustly
        float theta = (float) Math.atan2(Math.sqrt(Rx*Rx + Ry*Ry + Rz*Rz), (double) Rw);

        // compute interpolation variables
        float s0 = (float) Math.sin((1.0f - t)*theta);
        float s1 = (float) Math.sin(t*theta);

        // compute interpolated quaternion
        float Sw = s0*Aw + s1*Bw;
        float Sx = s0*Ax + s1*Bx;
        float Sy = s0*Ay + s1*By;
        float Sz = s0*Az + s1*Bz;

        // compute the length of the quaternion
        float mag = (float) Math.sqrt(Sw*Sw + Sx*Sx + Sy*Sy + Sz*Sz);

        if (mag > 0f) {
            float inv = 1f/mag;
            this.w = inv*Sw;
            this.x = inv*Sx;
            this.y = inv*Sy;
            this.z = inv*Sz;
        } else if (t >= 0.5f) {
            this.w = Bw;
            this.x = Bx;
            this.y = By;
            this.z = Bz;
        } else {
            this.w = Aw;
            this.x = Ax;
            this.y = Ay;
            this.z = Az;
        }

        return this;
    }

    public Quaternion slerp(Quaternion A, Quaternion B, float t) {
        return this.slerp(
            A.w, A.x, A.y, A.z,
            B.w, B.x, B.y, B.z,
            t
        );
    }

    public Quaternion slerp(Quaternion that, float t) {
        Quaternion output = new Quaternion();
        return output.slerp(
            this.w, this.x, this.y, this.z,
            that.w, that.x, that.y, that.z,
            t
        );
    }

    public Quaternion slerpNearest(
        float Aw, float Ax, float Ay, float Az,
        float Bw, float Bx, float By, float Bz,
        float t
    ) {
        if (Aw*Bw + Ax*Bx + Ay*By + Az*Bz < 0) {
            return this.slerp(
                -Aw, -Ax, -Ay, -Az,
                 Bw,  Bx,  By,  Bz,
                t
            );
        } else {
            return this.slerp(
                Aw, Ax, Ay, Az,
                Bw, Bx, By, Bz,
                t
            );
        }
    }

    public Quaternion slerpNearest(Quaternion A, Quaternion B, float t) {
        return this.slerpNearest(
            A.w, A.x, A.y, A.z,
            B.w, B.x, B.y, B.z,
            t
        );
    }

    public Quaternion slerpNearest(Quaternion that, float t) {
        Quaternion output = new Quaternion();
        return output.slerpNearest(
            this.w, this.x, this.y, this.z,
            that.w, that.x, that.y, that.z,
            t
        );
    }




    public Quaternion loadIdentity() {
        this.w = 1;
        this.x = 0;
        this.y = 0;
        this.z = 0;

        return this;
    }




    public String toString() {
        return w + " + " + x + " i + " + y + " j + " + z + " k"; // no ambiguity
    }
}
