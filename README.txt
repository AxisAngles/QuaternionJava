This is a work in progress. It is currently somewhat untested.
The Vector3f and Matrix3f classes provide minimum viable functionality.
Some methods are undocumented.

CONVERSIONS:
	fromRandom

	fromRotationMatrix
	toRotationMatrix

	fromRotationVector
	toRotationVector

	fromAngleAxis
	toAxis
	toAngle
	toAngleAxis

	fromEulerXYZ
	fromEulerXZY
	fromEulerYXZ
	fromEulerYZX
	fromEulerZXY
	fromEulerZYX
	toEulerXYZ
	toEulerYZX
	toEulerZXY
	toEulerZYX
	toEulerYXZ
	toEulerXZY
  
QUATERNION OPERATIONS:
	norm (will be renamed to lenSq)
	mag (will be renamed to len)
	unit
	neg
	conj
	inv

QUATERNION-SCALAR OPERATIONS:
	mul
	div

QUATERNION-QUATERNION OPERATIONS:
	dot
	add
	sub
	mul
	invMul
	mulInv

QUATERNION-VECTOR OPERATIONS:
	sandwich

PROJECTIONS:
	project
	projectUnitize
	projectedAngle
	align
	alignUnitize

INTERPOLATION:
	slerp
	slerpNearest

OTHER:
	angleBetween
	loadIdentity
	toString
