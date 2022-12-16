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
	// base functionality
	public float w, x, y, z;

	// constructors
	public Quaternion() {
		w = 1f;
		x = 0f;
		y = 0f;
		z = 0f;
	}

	public Quaternion(float Qw, float Qx, float Qy, float Qz) {
		w = Qw;
		x = Qx;
		y = Qy;
		z = Qz;
	}

	public Quaternion(Quaternion Q) {
		w = Q.w;
		x = Q.x;
		y = Q.y;
		z = Q.z;
	}

	public static Quaternion NULL = new Quaternion(0, 0, 0, 0);
	public static Quaternion IDENTITY = new Quaternion(1, 0, 0, 0);
	public static Quaternion I = new Quaternion(0, 1, 0, 0);
	public static Quaternion J = new Quaternion(0, 0, 1, 0);
	public static Quaternion K = new Quaternion(0, 0, 0, 1);

	public Quaternion set(float Qw, float Qx, float Qy, float Qz) {
		w = Qw;
		x = Qx;
		y = Qy;
		z = Qz;

		return this;
	}

	//Basic operations
	public float dot(Quaternion that) {
		return this.w*that.w + this.x*that.x + this.y*that.y + this.z*that.z;
	}
	public float lenSq() {
		return w*w + x*x + y*y + z*z;
	}
	public float len() {
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
	public Quaternion inv(Quaternion A) {
		float inv = 1f/(A.w*A.w + A.x*A.x + A.y*A.y + A.z*A.z);
		w = inv*A.w;
		x = inv*-A.x;
		y = inv*-A.y;
		z = inv*-A.z;

		return this;
	}
	public Quaternion mul(Quaternion A, float b) {
		w = A.w*b;
		x = A.x*b;
		y = A.y*b;
		z = A.z*b;

		return this;
	}
	public Quaternion div(Quaternion A, float b) {
		w = A.w/b;
		x = A.x/b;
		y = A.y/b;
		z = A.z/b;

		return this;
	}
	public Quaternion add(Quaternion A, Quaternion B) {
		w = A.w + B.w;
		x = A.x + B.x;
		y = A.y + B.y;
		z = A.z + B.z;

		return this;
	}
	public Quaternion sub(Quaternion A, Quaternion B) {
		w = A.w - B.w;
		x = A.x - B.x;
		y = A.y - B.y;
		z = A.z - B.z;

		return this;
	}
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

	//Projections
	public Quaternion project(Quaternion Q, float ax, float ay, float az) {
		float aLenSq = ax*ax + ay*ay + az*az;
		float aDotQ = Q.x*ax + Q.y*ay + Q.z*az;

		float t = aDotQ/aLenSq;
		w = Q.w;
		x = t*ax;
		y = t*ay;
		z = t*az;

		return this;
	}
	public Quaternion projectUnitize(Quaternion Q, float ax, float ay, float az) {
		return this.unit(this.project(Q, ax, ay, az));
	}
	public float projectedAngle(float ax, float ay, float az) {
		float aLen = (float) Math.sqrt(ax*ax + ay*ay + az*az);
		float aDotQ = x*ax + y*ay + z*az;

		float ang = (float) Math.atan2(aDotQ, w*aLen);
		return ang;
	}
	public Quaternion align(
		Quaternion Q,
		float ax, float ay, float az,
		float bx, float by, float bz
	) {
		float aLenSq = ax*ax + ay*ay + az*az;
		float bLenSq = bx*bx + by*by + bz*bz;

		float aLenSqInv = 1f/aLenSq;

		float Rw = aLenSqInv*(Q.x*ax + Q.y*ay + Q.z*az);
		float Rx = aLenSqInv*(Q.z*ay - Q.w*ax - Q.y*az);
		float Ry = aLenSqInv*(Q.x*az - Q.w*ay - Q.z*ax);
		float Rz = aLenSqInv*(Q.y*ax - Q.w*az - Q.x*ay);

		float Sw = -bx*Rx - by*Ry - bz*Rz;
		float Sx =  bx*Rw - bz*Ry + by*Rz;
		float Sy =  by*Rw + bz*Rx - bx*Rz;
		float Sz =  bz*Rw - by*Rx + bx*Ry;

		float mul = (float) Math.sqrt(aLenSqInv*bLenSq);

		// (b*Q*a^-1 + len(b*a^-1)*Q)/2, a and b are treated as pure imaginary quaternions
		w = 0.5f*(Sw + mul*Q.w);
		x = 0.5f*(Sx + mul*Q.x);
		y = 0.5f*(Sy + mul*Q.y);
		z = 0.5f*(Sz + mul*Q.z);

		return this;
	}
	public Quaternion alignUnitize(
		Quaternion Q,
		float ax, float ay, float az,
		float bx, float by, float bz
	) {
		return this.unit(this.align(Q, ax, ay, az, bx, by, bz));
	}

	// Interpolation
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
		float theta = (float) Math.atan2(Math.sqrt(Rx*Rx + Ry*Ry + Rz*Rz), Rw);

		// compute interpolation variables
		float s0 = (float) Math.sin((1.0f - t)*theta);
		float s1 = (float) Math.sin(t*theta);

		// compute interpolated quaternion
		float Sw = s0*Aw + s1*Bw;
		float Sx = s0*Ax + s1*Bx;
		float Sy = s0*Ay + s1*By;
		float Sz = s0*Az + s1*Bz;

		// compute the length of the quaternion (approximately sin(theta), but this is not robust)
		float len = (float) Math.sqrt(Sw*Sw + Sx*Sx + Sy*Sy + Sz*Sz);

		if (len > 0f) {
			float inv = 1f/len;
			w = inv*Sw;
			x = inv*Sx;
			y = inv*Sy;
			z = inv*Sz;
		} else if (t < 0.5f) {
			w = Aw;
			x = Ax;
			y = Ay;
			z = Az;
		} else {
			w = Bw;
			x = Bx;
			y = By;
			z = Bz;
		}

		return this;
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

	public float angleTo(Quaternion that) {
		float Rw = this.w*that.w + this.x*that.x + this.y*that.y + this.z*that.z;
		float Rx = this.w*that.x - this.x*that.w - this.y*that.z + this.z*that.y;
		float Ry = this.w*that.y + this.x*that.z - this.y*that.w - this.z*that.x;
		float Rz = this.w*that.z - this.x*that.y + this.y*that.x - this.z*that.w;

		// compute cosine and sine of the angle between
		// do so in a numerically stable way
		return (float) Math.atan2(Math.sqrt(Rx*Rx + Ry*Ry + Rz*Rz), Rw);
	}

//Quaternion Vector
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

// conversion from
	public Quaternion setFromRandom(float r0, float r1, float r2, float r3) {
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
	public Quaternion setFromRotationMatrix(
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
	public Quaternion setFromRotationVector(float rx, float ry, float rz) {
		float len = (float) Math.sqrt(rx*rx + ry*ry + rz*rz);
		if (len == 0f) {
			return new Quaternion();
		}

		float cos = (float) Math.cos(0.5f*len);
		float sin = (float) Math.sin(0.5f*len);
		float inv = 1f/len;
		
		w = cos;
		x = sin*inv*rx;
		y = sin*inv*ry;
		z = sin*inv*rz;

		return this;
	}
	public Quaternion setFromAngleAxis(float ang, float ax, float ay, float az) {
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

		w = cos;
		x = sin*inv*ax;
		y = sin*inv*ay;
		z = sin*inv*az;

		return this;
	}
	public Quaternion setFromEulerXYZ(float X, float Y, float Z) {
		float cosX = (float) Math.cos(0.5f*X);
		float cosY = (float) Math.cos(0.5f*Y);
		float cosZ = (float) Math.cos(0.5f*Z);
		float sinX = (float) Math.sin(0.5f*X);
		float sinY = (float) Math.sin(0.5f*Y);
		float sinZ = (float) Math.sin(0.5f*Z);

		w = cosX*cosY*cosZ - sinX*sinY*sinZ;
		x = cosY*cosZ*sinX + cosX*sinY*sinZ;
		y = cosX*cosZ*sinY - cosY*sinX*sinZ;
		z = cosZ*sinX*sinY + cosX*cosY*sinZ;

		return this;
	}
	public Quaternion setFromEulerXZY(float X, float Z, float Y) {
		float cosX = (float) Math.cos(0.5f*X);
		float cosY = (float) Math.cos(0.5f*Y);
		float cosZ = (float) Math.cos(0.5f*Z);
		float sinX = (float) Math.sin(0.5f*X);
		float sinY = (float) Math.sin(0.5f*Y);
		float sinZ = (float) Math.sin(0.5f*Z);

		w = cosX*cosY*cosZ + sinX*sinY*sinZ;
		x = cosY*cosZ*sinX - cosX*sinY*sinZ;
		y = cosX*cosZ*sinY - cosY*sinX*sinZ;
		z = cosZ*sinX*sinY + cosX*cosY*sinZ;

		return this;
	}
	public Quaternion setFromEulerYXZ(float Y, float X, float Z) {
		float cosX = (float) Math.cos(0.5f*X);
		float cosY = (float) Math.cos(0.5f*Y);
		float cosZ = (float) Math.cos(0.5f*Z);
		float sinX = (float) Math.sin(0.5f*X);
		float sinY = (float) Math.sin(0.5f*Y);
		float sinZ = (float) Math.sin(0.5f*Z);

		w = cosX*cosY*cosZ + sinX*sinY*sinZ;
		x = cosY*cosZ*sinX + cosX*sinY*sinZ;
		y = cosX*cosZ*sinY - cosY*sinX*sinZ;
		z = cosX*cosY*sinZ - cosZ*sinX*sinY;

		return this;
	}
	public Quaternion setFromEulerYZX(float Y, float Z, float X) {
		float cosX = (float) Math.cos(0.5f*X);
		float cosY = (float) Math.cos(0.5f*Y);
		float cosZ = (float) Math.cos(0.5f*Z);
		float sinX = (float) Math.sin(0.5f*X);
		float sinY = (float) Math.sin(0.5f*Y);
		float sinZ = (float) Math.sin(0.5f*Z);

		w = cosX*cosY*cosZ - sinX*sinY*sinZ;
		x = cosY*cosZ*sinX + cosX*sinY*sinZ;
		y = cosX*cosZ*sinY + cosY*sinX*sinZ;
		z = cosX*cosY*sinZ - cosZ*sinX*sinY;

		return this;
	}
	public Quaternion setFromEulerZXY(float Z, float X, float Y) {
		float cosX = (float) Math.cos(0.5f*X);
		float cosY = (float) Math.cos(0.5f*Y);
		float cosZ = (float) Math.cos(0.5f*Z);
		float sinX = (float) Math.sin(0.5f*X);
		float sinY = (float) Math.sin(0.5f*Y);
		float sinZ = (float) Math.sin(0.5f*Z);

		w = cosX*cosY*cosZ - sinX*sinY*sinZ;
		x = cosY*cosZ*sinX - cosX*sinY*sinZ;
		y = cosX*cosZ*sinY + cosY*sinX*sinZ;
		z = cosZ*sinX*sinY + cosX*cosY*sinZ;

		return this;
	}
	public Quaternion setFromEulerZYX(float Z, float Y, float X) {
		float cosX = (float) Math.cos(0.5f*X);
		float cosY = (float) Math.cos(0.5f*Y);
		float cosZ = (float) Math.cos(0.5f*Z);
		float sinX = (float) Math.sin(0.5f*X);
		float sinY = (float) Math.sin(0.5f*Y);
		float sinZ = (float) Math.sin(0.5f*Z);

		w = cosX*cosY*cosZ + sinX*sinY*sinZ;
		x = cosY*cosZ*sinX - cosX*sinY*sinZ;
		y = cosX*cosZ*sinY + cosY*sinX*sinZ;
		z = cosX*cosY*sinZ - cosZ*sinX*sinY;

		return this;
	}
	
	

// conversion to
	public Matrix3f toRotationMatrix(Matrix3f output) {
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
	
	private static float EULER_TOL = 10000f; // approximately tan(pi/2*0.9999)
	public float[] toEulerXYZ(float[] output) {
		float zz = w*w - x*x - y*y + z*z;
		float zy = 2f*(y*z - w*x);
		float kc = (float) Math.sqrt(zy*zy + zz*zz);
		float zx = 2f*(w*y + x*z);
		float xx = w*w + x*x - y*y - z*z;
		float yx = 2f*(x*y - w*z);

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
	public float[] toEulerYZX(float[] output) {
		float xx = w*w + x*x - y*y - z*z;
		float xz = 2f*(x*z - w*y);
		float kc = (float) Math.sqrt(xz*xz + xx*xx);
		float xy = 2f*(x*y + w*z);
		float yy = w*w - x*x + y*y - z*z;
		float zy = 2f*(y*z - w*x);

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
	public float[] toEulerZXY(float[] output) {
		float yy = w*w - x*x + y*y - z*z;
		float yx = 2f*(x*y - w*z);
		float kc = (float) Math.sqrt(yx*yx + yy*yy);
		float yz = 2f*(w*x + y*z);
		float zz = w*w - x*x - y*y + z*z;
		float xz = 2f*(x*z - w*y);

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
	public float[] toEulerZYX(float[] output) {
		float xx = w*w + x*x - y*y - z*z;
		float xy = 2f*(x*y + w*z);
		float kc = (float) Math.sqrt(xy*xy + xx*xx);
		float xz = 2f*(x*z - w*y);
		float zz = w*w - x*x - y*y + z*z;
		float yz = 2f*(w*x + y*z);

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
	public float[] toEulerYXZ(float[] output) {
		float zz = w*w - x*x - y*y + z*z;
		float zx = 2f*(w*y + x*z);
		float kc = (float) Math.sqrt(zx*zx + zz*zz);
		float zy = 2f*(y*z - w*x);
		float yy = w*w - x*x + y*y - z*z;
		float xy = 2f*(x*y + w*z);

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
	public float[] toEulerXZY(float[] output) {
		float yy = w*w - x*x + y*y - z*z;
		float yz = 2f*(w*x + y*z);
		float kc = (float) Math.sqrt(yz*yz + yy*yy);
		float yx = 2f*(x*y - w*z);
		float xx = w*w + x*x - y*y - z*z;
		float zx = 2f*(w*y + x*z);

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

	public Quaternion loadIdentity() {
		w = 1f;
		x = 0f;
		y = 0f;
		z = 0f;

		return this;
	}
	public String toString() {
		return w
			+ (x < 0 ? " - " + -x : " + " + x) + "*i"
			+ (y < 0 ? " - " + -y : " + " + y) + "*j"
			+ (z < 0 ? " - " + -z : " + " + z) + "*k"; // no ambiguity
	}

	//	alt arguments
	public Vector3f sandwich(Vector3f vector, Vector3f output) {return this.sandwich(vector.x, vector.y, vector.z, output);}
	public Quaternion project(Quaternion Q, Vector3f axis) {return this.project(Q, axis.x, axis.y, axis.z);}
	public Quaternion projectUnitize(Quaternion Q, Vector3f axis) {return this.projectUnitize(Q, axis.x, axis.y, axis.z);}
	public float projectedAngle(Vector3f axis) {return this.projectedAngle(axis.x, axis.y, axis.z);}
	public Quaternion align(Quaternion Q, Vector3f a, Vector3f b) {return this.align(Q, a.x, a.y, a.z, b.x, b.y, b.z);}
	public Quaternion alignUnitize(Quaternion Q, Vector3f a, Vector3f b) {return this.alignUnitize(Q, a.x, a.y, a.z, b.x, b.y, b.z);}
	public Quaternion slerp(Quaternion A, Quaternion B, float t) {return this.slerp(A.w, A.x, A.y, A.z, B.w, B.x, B.y, B.z, t);}
	public Quaternion slerpNearest(Quaternion A, Quaternion B, float t) {return this.slerp(A.w, A.x, A.y, A.z, B.w, B.x, B.y, B.z, t);}
	public Quaternion setFromRotationMatrix(Matrix3f matrix) {return this.setFromRotationMatrix(
		matrix.m00, matrix.m01, matrix.m02,
		matrix.m10, matrix.m11, matrix.m12,
		matrix.m20, matrix.m21, matrix.m22
	);}
	public Quaternion setFromRotationVector(Vector3f vector) {return this.setFromRotationVector(vector.x, vector.y, vector.z);}
	public Quaternion setFromAngleAxis(float ang, Vector3f axis) {return this.setFromAngleAxis(ang, axis.x, axis.y,axis.z);}
	public Quaternion setFromEulerXYZ(float[] angles) {return this.setFromEulerXYZ(angles[0], angles[1], angles[2]);}
	public Quaternion setFromEulerXZY(float[] angles) {return this.setFromEulerXZY(angles[0], angles[1], angles[2]);}
	public Quaternion setFromEulerYXZ(float[] angles) {return this.setFromEulerYXZ(angles[0], angles[1], angles[2]);}
	public Quaternion setFromEulerYZX(float[] angles) {return this.setFromEulerYZX(angles[0], angles[1], angles[2]);}
	public Quaternion setFromEulerZXY(float[] angles) {return this.setFromEulerZXY(angles[0], angles[1], angles[2]);}
	public Quaternion setFromEulerZYX(float[] angles) {return this.setFromEulerZYX(angles[0], angles[1], angles[2]);}









	//	immutable conversion shorthand
	//	calls of the form:
	//		!Quaternion result = quaternion.toF(new !Quaternion);
	//	become
	//		!Quaternion result = quaternion.toF();
	public Matrix3f toRotationMatrix() {return this.toRotationMatrix(new Matrix3f());}
	public Vector3f toRotationVector() {return this.toRotationVector(new Vector3f());}
	public Vector3f toAxis() {return this.toAxis(new Vector3f());}
	public float toAngleAxis() {return this.toAngleAxis(new Vector3f());}
	public float[] toEulerXYZ() {return this.toEulerXYZ(new float[3]);}
	public float[] toEulerYZX() {return this.toEulerYZX(new float[3]);}
	public float[] toEulerZXY() {return this.toEulerZXY(new float[3]);}
	public float[] toEulerZYX() {return this.toEulerZYX(new float[3]);}
	public float[] toEulerYXZ() {return this.toEulerYXZ(new float[3]);}
	public float[] toEulerXZY() {return this.toEulerXZY(new float[3]);}











	//	immutable shorthand
	//	calls of the form:
	//		Quaternion result = new Quaternion().f(Quaternion a, ...);
	//	become
	//		Quaternion result = a.f(...);
	public Quaternion unit() {return new Quaternion().unit(this);}
	public Quaternion neg() {return new Quaternion().neg(this);}
	public Quaternion conj() {return new Quaternion().conj(this);}
	public Quaternion inv() {return new Quaternion().inv(this);}
	public Quaternion mul(float that) {return new Quaternion().mul(this, that);}
	public Quaternion div(float that) {return new Quaternion().div(this, that);}
	public Quaternion add(Quaternion that) {return new Quaternion().add(this, that);}
	public Quaternion sub(Quaternion that) {return new Quaternion().sub(this, that);}
	public Quaternion mul(Quaternion that) {return new Quaternion().mul(this, that);}
	public Quaternion invMul(Quaternion that) {return new Quaternion().invMul(this, that);}
	public Quaternion mulInv(Quaternion that) {return new Quaternion().mulInv(this, that);}
	public Quaternion project(float ax, float ay, float az) {return new Quaternion().project(this, ax, ay, az);}
	public Quaternion projectUnitize(float ax, float ay, float az) {return new Quaternion().projectUnitize(this, ax, ay, az);}
	public Quaternion align(
		float ax, float ay, float az,
		float bx, float by, float bz
	) {return new Quaternion().align(this, ax, ay, az, bx, by, bz);}
	public Quaternion alignUnitize(
		float ax, float ay, float az,
		float bx, float by, float bz
	) {return new Quaternion().alignUnitize(this, ax, ay, az, bx, by, bz);}

	//	alt arguments
	public Quaternion project(Vector3f axis) {return new Quaternion().project(this, axis);}
	public Quaternion projectUnitize(Vector3f axis) {return new Quaternion().projectUnitize(this, axis);}
	public Quaternion align(Vector3f a, Vector3f b) {return new Quaternion().align(this, a, b);}
	public Quaternion alignUnitize(Vector3f a, Vector3f b) {return new Quaternion().alignUnitize(this, a, b);}
	public Quaternion slerp(Quaternion that, float t) {return new Quaternion().slerp(this, that, t);}
	public Quaternion slerpNearest(Quaternion that, float t) {return new Quaternion().slerpNearest(this, that, t);}





	//	static shorthand
	//	calls of the form:
	//		Quaternion result = new Quaternion().setF(!Quaternion ...);
	//	become
	//		Quaternion result = Quaternion.f(!Quaternion ...);
	public static Quaternion fromRandom(float r0, float r1, float r2, float r3) {return new Quaternion().setFromRandom(r0, r1, r2, r3);}
	public static Quaternion fromRotationMatrix(
		float xx, float yx, float zx,
		float xy, float yy, float zy,
		float xz, float yz, float zz
	) {return new Quaternion().setFromRotationMatrix(xx, yx, zx, xy, yy, zy, xz, yz, zz);};
	public static Quaternion fromRotationVector(float rx, float ry, float rz) {return new Quaternion().setFromRotationVector(rx, ry, rz);}
	public static Quaternion fromAngleAxis(float ang, float ax, float ay, float az) {return new Quaternion().setFromAngleAxis(ang, ax, ay, az);}
	public static Quaternion fromEulerXYZ(float X, float Y, float Z) {return new Quaternion().setFromEulerXYZ(X, Y, Z);}
	public static Quaternion fromEulerXZY(float X, float Z, float Y) {return new Quaternion().setFromEulerXZY(X, Z, Y);}
	public static Quaternion fromEulerYXZ(float Y, float X, float Z) {return new Quaternion().setFromEulerYXZ(Y, X, Z);}
	public static Quaternion fromEulerYZX(float Y, float Z, float X) {return new Quaternion().setFromEulerYZX(Y, Z, X);}
	public static Quaternion fromEulerZXY(float Z, float X, float Y) {return new Quaternion().setFromEulerZXY(Z, X, Y);}
	public static Quaternion fromEulerZYX(float Z, float Y, float X) {return new Quaternion().setFromEulerZYX(Z, Y, X);}

	//	immutable shorthand alt arguments
	public static Quaternion fromRotationMatrix(Matrix3f matrix) {return new Quaternion().setFromRotationMatrix(matrix);}
	public static Quaternion fromRotationVector(Vector3f vector) {return new Quaternion().setFromRotationVector(vector);}
	public static Quaternion fromAngleAxis(float ang, Vector3f axis) {return new Quaternion().setFromAngleAxis(ang, axis);}
	public static Quaternion fromEulerXYZ(float[] angles) {return new Quaternion().setFromEulerXYZ(angles);}
	public static Quaternion fromEulerXZY(float[] angles) {return new Quaternion().setFromEulerXZY(angles);}
	public static Quaternion fromEulerYXZ(float[] angles) {return new Quaternion().setFromEulerYXZ(angles);}
	public static Quaternion fromEulerYZX(float[] angles) {return new Quaternion().setFromEulerYZX(angles);}
	public static Quaternion fromEulerZXY(float[] angles) {return new Quaternion().setFromEulerZXY(angles);}
	public static Quaternion fromEulerZYX(float[] angles) {return new Quaternion().setFromEulerZYX(angles);}





	//	updater shorthand
	//	when this is unambiguously being acted upon
	//	calls of the form
	//		Quaternion this = this.f(this, ...);
	//	become
	//		Quaternion this = this.f(...);
	public Quaternion unitThis() {return this.unit(this);}
	public Quaternion negThis() {return this.neg(this);}
	public Quaternion conjThis() {return this.conj(this);}
	public Quaternion invThis() {return this.inv(this);}
	public Quaternion mulThis(float that) {return this.mul(this, that);}
	public Quaternion divThis(float that) {return this.div(this, that);}
	public Quaternion projectThis(float ax, float ay, float az) {return this.project(this, ax, ay, az);}
	public Quaternion projectUnitizeThis(float ax, float ay, float az) {return this.projectUnitize(this, ax, ay, az);}
	public Quaternion alignThis(
		float ax, float ay, float az,
		float bx, float by, float bz
	) {return this.align(this, ax, ay, az, bx, by, bz);}
	public Quaternion alignUnitizeThis(
		float ax, float ay, float az,
		float bx, float by, float bz
	) {return this.alignUnitize(this, ax, ay, az, bx, by, bz);}
	// public Quaternion slerpThis(Quaternion that, float t) {return this.slerp(this, that, t);}
	// public Quaternion slerpThat(Quaternion that, float t) {return that.slerp(this, that, t);}
	// public Quaternion slerpNearestThis(Quaternion that, float t) {return this.slerpNearest(this, that, t);}
	// public Quaternion slerpNearestThat(Quaternion that, float t) {return that.slerpNearest(this, that, t);}
	// public Quaternion mulThis(Quaternion that) {return this.mul(this, that);}
	// public Quaternion mulThat(Quaternion that) {return that.mul(this, that);}
	// public Quaternion addThis(Quaternion that) {return this.add(this, that);}
	// public Quaternion addThat(Quaternion that) {return that.add(this, that);} // not necessary
	// public Quaternion subThis(Quaternion that) {return this.sub(this, that);}
	// public Quaternion subThat(Quaternion that) {return that.sub(this, that);}

	//	updater shorthand alt arguments
	public Quaternion projectThis(Vector3f axis) {return this.project(this, axis);}
	public Quaternion projectUnitizeThis(Vector3f axis) {return this.projectUnitize(this, axis);}
	public Quaternion alignThis(Vector3f a, Vector3f b) {return this.align(this, a, b);}
	public Quaternion alignUnitizeThis(Vector3f a, Vector3f b) {return this.alignUnitize(this, a, b);}








	//	varargs shorthand
	//	for chain-able operations of the form Quaternion result = new Quaternion().f(Quaternion ...)
	public Quaternion adds(Quaternion... args) {
		Quaternion result = new Quaternion(this);
		for (int i = 0; i < args.length; i++) {
			result.add(result, args[i]);
		}
		return result;
	}

	public Quaternion muls(Quaternion... args) {
		Quaternion result = new Quaternion(this);
		for (int i = 0; i < args.length; i++) {
			result.mul(result, args[i]);
		}
		return result;
	}
}
