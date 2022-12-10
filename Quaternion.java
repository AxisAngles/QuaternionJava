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

// I want this to be easy to use and hard to shoot yourself in the foot
// While also being minimal and not super slow

public final class Quaternion {
	public float w, x, y, z;

	public Quaternion() {
		w = 1;
		x = 0;
		y = 0;
		z = 0;
	}

	public Quaternion(float w, float x, float y, float z) {
		this.w = w;
		this.x = x;
		this.y = y;
		this.z = z;
	}

	public Quaternion(Quaternion Q) {
		w = Q.w;
		x = Q.x;
		y = Q.y;
		z = Q.z;
	}



	// non-standard matrix naming
	// xy means xVector's yComponent
	// Please give it an unscaled orthogonal (rotation) matrix.
	// Orthogonalization can be done with R*inverse(sqrt(transpose(R)*R))
	public Quaternion fromRotationMatrixLocal(
		float xx, float yx, float zx,
		float xy, float yy, float zy,
		float xz, float yz, float zz
	) {
		float Qw, Qx, Qy, Qz;
		if (yy > -zz && zz > -xx && xx > -yy) {
			Qw = 1 + xx + yy + zz;
			Qx = yz - zy;
			Qy = zx - xz;
			Qz = xy - yx;
		} else if (xx > yy && xx > zz) {
			Qw = yz - zy;
			Qx = 1 + xx - yy - zz;
			Qy = xy + yx;
			Qz = xz + zx;
		} else if (yy > zz) {
			Qw = zx - xz;
			Qx = xy + yx;
			Qy = 1 - xx + yy - zz;
			Qz = yz + zy;
		} else {
			Qw = xy - yx;
			Qx = xz + zx;
			Qy = yz + zy;
			Qz = 1 - xx - yy + zz;
		}

		// Hold off on unitization until the end
		float inv = 1/(float) Math.sqrt(Qw*Qw + Qx*Qx + Qy*Qy + Qz*Qz);
		this.w = inv*Qw;
		this.x = inv*Qx;
		this.y = inv*Qy;
		this.z = inv*Qz;

		return this;
	}

	public Quaternion fromRotationMatrixLocal(Matrix3f matrix) {
		return this.fromRotationMatrixLocal(
			matrix.m00, matrix.m01, matrix.m02,
			matrix.m10, matrix.m11, matrix.m12,
			matrix.m20, matrix.m21, matrix.m22
		);
	}

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

	public static Quaternion fromRotationMatrix(Matrix3f matrix) {
		Quaternion output = new Quaternion();
		return output.fromRotationMatrixLocal(
			matrix.m00, matrix.m01, matrix.m02,
			matrix.m10, matrix.m11, matrix.m12,
			matrix.m20, matrix.m21, matrix.m22
		);
	}




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

	public Quaternion fromRotationVectorLocal(Vector3f rotationVector) {
		return this.fromRotationVectorLocal(
			rotationVector.x,
			rotationVector.y,
			rotationVector.z
		);
	}

	public static Quaternion fromRotationVector(float rx, float ry, float rz) {
		Quaternion output = new Quaternion();
		return output.fromRotationVectorLocal(rx, ry, rz);
	}

	public static Quaternion fromRotationVector(Vector3f rotationVector) {
		Quaternion output = new Quaternion();
		return output.fromRotationVectorLocal(
			rotationVector.x,
			rotationVector.y,
			rotationVector.z
		);
	}

	public Quaternion fromAngleAxisLocal(float ang, float ax, float ay, float az) {
		float len = (float) Math.sqrt(ax*ax + ay*ay + az*az);
		if (len == 0f) {
			return new Quaternion(); // technically not defined but sure
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

	public Quaternion fromAngleAxisLocal(float ang, Vector3f axis) {
		return this.fromAngleAxisLocal(ang, axis.x, axis.y, axis.z);
	}

	public static Quaternion fromAngleAxis(float ang, float ax, float ay, float az) {
		Quaternion output = new Quaternion();
		return output.fromAngleAxisLocal(ang, ax, ay, az);
	}

	public static Quaternion fromAngleAxis(float ang, Vector3f axis) {
		Quaternion output = new Quaternion();
		return output.fromAngleAxisLocal(ang, axis.x, axis.y, axis.z);
	}




	public static Quaternion fromEulerXYZ(float X, float Y, float Z) {
		Quaternion output = new Quaternion();
		return output.fromEulerXYZLocal(X, Y, Z);
	}

	public static Quaternion fromEulerXZY(float X, float Z, float Y) {
		Quaternion output = new Quaternion();
		return output.fromEulerXZYLocal(X, Z, Y);
	}

	public static Quaternion fromEulerYXZ(float Y, float X, float Z) {
		Quaternion output = new Quaternion();
		return output.fromEulerYXZLocal(Y, X, Z);
	}

	public static Quaternion fromEulerYZX(float Y, float Z, float X) {
		Quaternion output = new Quaternion();
		return output.fromEulerYZXLocal(Y, Z, X);
	}

	public static Quaternion fromEulerZXY(float Z, float X, float Y) {
		Quaternion output = new Quaternion();
		return output.fromEulerZXYLocal(Z, X, Y);
	}

	public static Quaternion fromEulerZYX(float Z, float Y, float X) {
		Quaternion output = new Quaternion();
		return output.fromEulerZYXLocal(Z, Y, X);
	}




	public static Quaternion fromEulerXYZ(float[] angles) {
		Quaternion output = new Quaternion();
		return output.fromEulerXYZLocal(angles[0], angles[1], angles[2]);
	}

	public static Quaternion fromEulerXZY(float[] angles) {
		Quaternion output = new Quaternion();
		return output.fromEulerXZYLocal(angles[0], angles[1], angles[2]);
	}

	public static Quaternion fromEulerYXZ(float[] angles) {
		Quaternion output = new Quaternion();
		return output.fromEulerYXZLocal(angles[0], angles[1], angles[2]);
	}

	public static Quaternion fromEulerYZX(float[] angles) {
		Quaternion output = new Quaternion();
		return output.fromEulerYZXLocal(angles[0], angles[1], angles[2]);
	}

	public static Quaternion fromEulerZXY(float[] angles) {
		Quaternion output = new Quaternion();
		return output.fromEulerZXYLocal(angles[0], angles[1], angles[2]);
	}

	public static Quaternion fromEulerZYX(float[] angles) {
		Quaternion output = new Quaternion();
		return output.fromEulerZYXLocal(angles[0], angles[1], angles[2]);
	}


	// Array is in angles application order
	public float[] toEulerXYZ() {
		float[] output = new float[3];
		return this.toEulerXYZ(output);
	}

	public float[] toEulerXZY() {
		float[] output = new float[3];
		return this.toEulerXZY(output);
	}

	public float[] toEulerYXZ() {
		float[] output = new float[3];
		return this.toEulerYXZ(output);
	}

	public float[] toEulerYZX() {
		float[] output = new float[3];
		return this.toEulerYZX(output);
	}

	public float[] toEulerZXY() {
		float[] output = new float[3];
		return this.toEulerZXY(output);
	}

	public float[] toEulerZYX() {
		float[] output = new float[3];
		return this.toEulerZYX(output);
	}




	public Quaternion fromEulerXYZLocal(float[] angles) {
		return this.fromEulerXYZLocal(angles[0], angles[1], angles[2]);
	}

	public Quaternion fromEulerXZYLocal(float[] angles) {
		return this.fromEulerXZYLocal(angles[0], angles[1], angles[2]);
	}

	public Quaternion fromEulerYXZLocal(float[] angles) {
		return this.fromEulerYXZLocal(angles[0], angles[1], angles[2]);
	}

	public Quaternion fromEulerYZXLocal(float[] angles) {
		return this.fromEulerYZXLocal(angles[0], angles[1], angles[2]);
	}

	public Quaternion fromEulerZXYLocal(float[] angles) {
		return this.fromEulerZXYLocal(angles[0], angles[1], angles[2]);
	}

	public Quaternion fromEulerZYXLocal(float[] angles) {
		return this.fromEulerZYXLocal(angles[0], angles[1], angles[2]);
	}





	// EULER DEFINITIONS

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

	public float[] toEulerXYZ(float[] output) {
		float kCosX = w*w - x*x - y*y + z*z;
		float kCosZ = w*w + x*x - y*y - z*z;
		float kSinX = 2*(w*x - y*z);
		float kSinY = 2*(w*y + x*z);
		float kSinZ = 2*(w*z - x*y);
		float k = w*w + x*x + y*y + z*z;

		float X, Y, Z;
		if (kCosX == 0 && kSinX == 0 || kCosZ == 0 && kSinZ == 0) {
			// prefer first
			X = 2f*(float) Math.atan2(x, w);
			Y = (float) Math.asin(kSinY/k);
			Z = 0f;
		} else {
			X = (float) Math.atan2(kSinX, kCosX);
			Y = (float) Math.asin(kSinY/k);
			Z = (float) Math.atan2(kSinZ, kCosZ);
		}

		output[0] = X;
		output[1] = Y;
		output[2] = Z;

		return output;
	}

	public float[] toEulerXZY(float[] output) {
		float kCosX = w*w - x*x + y*y - z*z;
		float kCosY = w*w + x*x - y*y - z*z;
		float kSinX = 2*(w*x + y*z);
		float kSinZ = 2*(w*z - x*y);
		float kSinY = 2*(w*y + x*z);
		float k = w*w + x*x + y*y + z*z;

		float X, Z, Y;
		if (kCosX == 0 && kSinX == 0 || kCosY == 0 && kSinY == 0) {
			// prefer first
			X = 2f*(float) Math.atan2(x, w);
			Z = (float) Math.asin(kSinZ/k);
			Y = 0f;
		} else {
			X = (float) Math.atan2(kSinX, kCosX);
			Z = (float) Math.asin(kSinZ/k);
			Y = (float) Math.atan2(kSinY, kCosY);
		}

		output[0] = X;
		output[1] = Z;
		output[2] = Y;
		
		return output;
	}

	public float[] toEulerYXZ(float[] output) {
		float kCosY = w*w - x*x - y*y + z*z;
		float kCosZ = w*w - x*x + y*y - z*z;
		float kSinY = 2*(w*y + x*z);
		float kSinX = 2*(w*x - y*z);
		float kSinZ = 2*(x*y + w*z);
		float k = w*w + x*x + y*y + z*z;

		float Y, X, Z;
		if (kCosY == 0 && kSinY == 0 || kCosZ == 0 && kSinZ == 0) {
			// prefer first
			Y = 2f*(float) Math.atan2(y, w);
			X = (float) Math.asin(kSinX/k);
			Z = 0f;
		} else {
			Y = (float) Math.atan2(kSinY, kCosY);
			X = (float) Math.asin(kSinX/k);
			Z = (float) Math.atan2(kSinZ, kCosZ);
		}

		output[0] = Y;
		output[1] = X;
		output[2] = Z;
		
		return output;
	}

	public float[] toEulerYZX(float[] output) {
		float kCosY = w*w + x*x - y*y - z*z;
		float kCosX = w*w - x*x + y*y - z*z;
		float kSinY = 2*(w*y - x*z);
		float kSinZ = 2*(x*y + w*z);
		float kSinX = 2*(w*x - y*z);
		float k = w*w + x*x + y*y + z*z;

		float Y, Z, X;
		if (kCosY == 0 && kSinY == 0 || kCosX == 0 && kSinX == 0) {
			// prefer first
			Y = 2f*(float) Math.atan2(y, w);
			Z = (float) Math.asin(kSinZ/k);
			X = 0f;
		} else {
			Y = (float) Math.atan2(kSinY, kCosY);
			Z = (float) Math.asin(kSinZ/k);
			X = (float) Math.atan2(kSinX, kCosX);
		}

		output[0] = Y;
		output[1] = Z;
		output[2] = X;
		
		return output;
	}

	public float[] toEulerZXY(float[] output) {
		float kCosZ = w*w - x*x + y*y - z*z;
		float kCosY = w*w - x*x - y*y + z*z;
		float kSinZ = 2*(w*z - x*y);
		float kSinX = 2*(w*x + y*z);
		float kSinY = 2*(w*y - x*z);
		float k = w*w + x*x + y*y + z*z;

		float Z, X, Y;
		if (kCosZ == 0 && kSinZ == 0 || kCosY == 0 && kSinY == 0) {
			// prefer first
			Z = 2f*(float) Math.atan2(z, w);
			X = (float) Math.asin(kSinX/k);
			Y = 0f;
		} else {
			Z = (float) Math.atan2(kSinZ, kCosZ);
			X = (float) Math.asin(kSinX/k);
			Y = (float) Math.atan2(kSinY, kCosY);
		}

		output[0] = Z;
		output[1] = X;
		output[2] = Y;
		
		return output;
	}

	public float[] toEulerZYX(float[] output) {
		float kCosZ = w*w + x*x - y*y - z*z;
		float kCosX = w*w - x*x - y*y + z*z;
		float kSinZ = 2*(x*y + w*z);
		float kSinY = 2*(w*y - x*z);
		float kSinX = 2*(w*x + y*z);
		float k = w*w + x*x + y*y + z*z;

		float Z, Y, X;
		if (kCosZ == 0 && kSinZ == 0 || kCosX == 0 && kSinX == 0) {
			// prefer first
			Z = 2f*(float) Math.atan2(z, w);
			Y = (float) Math.asin(kSinY/k);
			X = 0f;
		} else {
			Z = (float) Math.atan2(kSinZ, kCosZ);
			Y = (float) Math.asin(kSinY/k);
			X = (float) Math.atan2(kSinX, kCosX);
		}

		output[0] = Z;
		output[1] = Y;
		output[2] = X;
		
		return output;
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
		Vector3 output = new Vector3f();
		return this.sandwich(vx, vy, vz, output);
	}

	public Vector3f sandwich(Vector3f vector) {
		Vector3 output = new Vector3f();
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
		return self.projectedAngle(axis.x, axis.y, axis.z);
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
