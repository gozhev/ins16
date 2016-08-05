#include <stdio.h>

#include <math.h>
#include "engine.h"

#define PHYS_G 9.80665 /* acceleration of gravity in meters per second squared */
/* Full Scale Ranges of Sensors */
#define ACCEL_FS 2 /* acceleration(s) of gravity */
#define GYRO_FS 500.0 /* degrees per sec */
/* software low-pass filter frame size */
#define SW_LOWPASS_SMPLCNT_A 500 /* samples */
#define SW_LOWPASS_SMPLCNT_W 500 /* samples */

/* convert raw values to SI */
static inline double
raw2si_w (double w)
{
	return (M_PI * GYRO_FS / 180.0) * (w / 32768.0);
}

static inline double
raw2si_a (double a) 
{
	return PHYS_G * ACCEL_FS * (a / 32768.0);
}

static inline int obtain_sample (int16_t *, int16_t *, int16_t *,
		int16_t *, int16_t *, int16_t *);

/* raw sample */
static int16_t rai, raj, rak, rwi, rwj, rwk;

/* local axes: i, j, k,
   global: x, y, z 

   a - acceleration
   w - angular velocity
   v - linear velocity
   r - position vector
   s - orientation quaternion

   *x - projection to x
   zo* - zero-point offset
   av* - average value
   _* - previous value
   *0..*3 - quaternion coordinates */
static double ai, aj, ak;
static double wi, wj, wk;

/* software low-pass filter */
static double lp_Ai[SW_LOWPASS_SMPLCNT_A] = {0.};
static double lp_Aj[SW_LOWPASS_SMPLCNT_A] = {0.};
static double lp_Ak[SW_LOWPASS_SMPLCNT_A] = {0.};
static double lp_Wi[SW_LOWPASS_SMPLCNT_W] = {0.};
static double lp_Wj[SW_LOWPASS_SMPLCNT_W] = {0.};
static double lp_Wk[SW_LOWPASS_SMPLCNT_W] = {0.};
static double lp_ai = 0.;
static double lp_aj = 0.;
static double lp_ak = 0.;
static double lp_wi = 0.;
static double lp_wj = 0.;
static double lp_wk = 0.;
static int lp_pos_a = 0;
static int lp_pos_w = 0;

static double ax, ay, az;

static double  vx,  vy,  vz;
static double _vx, _vy, _vz;

static double  rx,  ry,  rz;
static double _rx, _ry, _rz;

/* orientation quaternion:
   s0 = Cos (PHY/2)
   s1 = u * Sin (PHY/2)
   s2 = u * Sin (PHY/2)
   s3 = u * Sin (PHY/2)

   where u is a three-dimensional unit vector,
   and PHY is a rotation angle

   s0..3 represents rotation around the axis u of angle PHY */
static double  s0,  s1,  s2,  s3;
static double _s0, _s1, _s2, _s3;

static double q0, q1, q2, q3;

static double dt;

static double av_ai, av_aj, av_ak;

static int o_rwi, o_rwj, o_rwk;
static int o_rai, o_raj, o_rak;

static double f_ai, f_aj, f_ak; 

static double calibration_time = 0.;

static int smpl_cnt, clbr_cnt;

static int
calibrate (void)
{
	int err;
	/* TODO: 
	   calibration steps nedded:
	   - zero offset for accel
	   - sensitivity scaling for accel
	   - zero offset for gyro */

	/* I think we can not calibrate zero-point offset of accelerometer
	   in-place in static position. Do do so we need to rotate sensor
	   180 degrees around the axe which is orthogonal to acceleration
	   of gravity during the calibration process.
	   
	   Now I do this by hand: */
	o_rai = 775; /* 775 */
	o_raj = -225; /* -200 */
	o_rak = -875; /* -875 */

	f_ai = .9975; /* 0.9975; */
	f_aj = .9975; /* 0.9975; */
	f_ak = 0.9738; /* 0.9738; */

	o_rwi = o_rwj = o_rwk = 0;
	av_ai = av_aj = av_ak = 0.;

	clbr_cnt = calibration_time * SAMPLING_FREQUENCY;

	/* calibration loop */
	while (smpl_cnt < clbr_cnt) {
		err = obtain_sample (&rai, &raj, &rak, &rwi, &rwj, &rwk);
		if (err)
			return err;

		ai = raw2si_a (rai - o_rai) * f_ai;
		aj = raw2si_a (raj - o_raj) * f_aj;
		ak = raw2si_a (rak - o_rak) * f_ak;
		
		/* TODO: add averaging filter that uniforms dispersion, not average */


		av_ai += ai;
		av_aj += aj;
		av_ak += ak;
		o_rwi += rwi;
		o_rwj += rwj;
		o_rwk += rwk;

		smpl_cnt++;
	}

	av_ai /= smpl_cnt;
	av_aj /= smpl_cnt;
	av_ak /= smpl_cnt;
	o_rwi /= smpl_cnt;
	o_rwj /= smpl_cnt;
	o_rwk /= smpl_cnt;

	return 0;
}

int
engine_calculate (void)
{
	int rc;
	FILE *fout;

	fout = fopen ("out.dat", "w");
	if (fout == 0) {
		fprintf (stderr, "Failed to open %s for writing.\n", "out.dat");
		return -1;
	}

	smpl_cnt = 0;

	calibrate ();
	
	fprintf (stdout, "Averaging: %lf %lf %lf N: %lf\n", 
		 av_ai, av_aj, av_ak, 
		 sqrt (av_ai*av_ai + av_aj*av_aj + av_ak*av_ak));

	/* Okay, suppose all of calibrations are done.
	   First we need to determine global axes x, y and z.

	   Suppose the sensor is placed not very accurate, so there are
	   non-zero projections of vector (-g) to the local axes i, j, k
	   (note that accelerometer shows (-g), not (g) vector).

	   Introduce global vector z as a unit vector opposite to gravity
	   vector g: z = (-g)/|g|

	   So there are non-zero angle between vectors k and z.

	   The initial state of the orientation quaternion s0..3 is
	   the rotation from z to k around z-cross-k vector.

	   Introduce the axes x and y as a vectors which obtained by
	   rotating vectors i and j with initial quaternion s0..3.

	   Let's calculate that. */

	/* in the theory this should be equal to the acceleration of gravity */
	double av_g;
	av_g = sqrt ((av_ai * av_ai) + (av_aj * av_aj) + (av_ak * av_ak));

	/* Identity quaternion.
	   this is default value, it only true if our sensor is set absolutly
	   horizontally */
	_s0 = 1.;
	_s1 = _s2 = _s3 = 0.;

	/* let's find a quaternion corresponding to our real system
	   k has the coordinates 0, 0, 1 in the local system,
	   z has the coordinates zi, zj, zk in the local system.
	   To get transformation quaternion from ijk to xyz we need to
	   construct rotation quaternion from z to k.
	   NOTE: See ref/math directory for more details. */
	_s0 = sqrt (2. + (2. * (av_ak / av_g)));
	_s1 = (av_aj / av_g) / _s0;
	_s2 = -(av_ai / av_g) / _s0;
	_s3 = 0.;
	_s0 *= 0.5;

	_vx = _vy = _vz = 0.;
	_rx = _ry = _rz = 0.;

	/* prepare low-pass filter initial values */
	int i;
	for (i = 0; i < SW_LOWPASS_SMPLCNT_A; i++) {
		lp_Ai[i] = av_ai / SW_LOWPASS_SMPLCNT_A;
		lp_Aj[i] = av_aj / SW_LOWPASS_SMPLCNT_A;
		lp_Ak[i] = av_ak / SW_LOWPASS_SMPLCNT_A;
	}
	lp_ai = av_ai;
	lp_aj = av_aj;
	lp_ak = av_ak;

	dt = 1. / SAMPLING_FREQUENCY;

	while (1) {
		if (obtain_sample (&rai, &raj, &rak, &rwi, &rwj, &rwk))
			break;

		ai = raw2si_a (rai - o_rai) * f_ai;
		aj = raw2si_a (raj - o_raj) * f_aj;
		ak = raw2si_a (rak - o_rak) * f_ak;
		wi = raw2si_w (rwi - o_rwi);
		wj = raw2si_w (rwj - o_rwj);
		wk = raw2si_w (rwk - o_rwk);

		/* software low-pass filter */
		lp_ai -= lp_Ai[lp_pos_a];
		lp_aj -= lp_Aj[lp_pos_a];
		lp_ak -= lp_Ak[lp_pos_a];
		lp_wi -= lp_Wi[lp_pos_w];
		lp_wj -= lp_Wj[lp_pos_w];
		lp_wk -= lp_Wk[lp_pos_w];
		lp_Ai[lp_pos_a] = ai / SW_LOWPASS_SMPLCNT_A;
		lp_Aj[lp_pos_a] = aj / SW_LOWPASS_SMPLCNT_A;
		lp_Ak[lp_pos_a] = ak / SW_LOWPASS_SMPLCNT_A;
		lp_Wi[lp_pos_w] = wi / SW_LOWPASS_SMPLCNT_W;
		lp_Wj[lp_pos_w] = wj / SW_LOWPASS_SMPLCNT_W;
		lp_Wk[lp_pos_w] = wk / SW_LOWPASS_SMPLCNT_W;
		lp_ai += lp_Ai[lp_pos_a];
		lp_aj += lp_Aj[lp_pos_a];
		lp_ak += lp_Ak[lp_pos_a];
		lp_wi += lp_Wi[lp_pos_w];
		lp_wj += lp_Wj[lp_pos_w];
		lp_wk += lp_Wk[lp_pos_w];
		lp_pos_a++;
		if (lp_pos_a == SW_LOWPASS_SMPLCNT_A)
			lp_pos_a = 0;
		lp_pos_w++;
		if (lp_pos_w == SW_LOWPASS_SMPLCNT_W)
			lp_pos_w = 0;
		
		/* apply low-pass filter result */
		ai = lp_ai;
		aj = lp_aj;
		ak = lp_ak;
		/*wi = lp_wi;
		wj = lp_wj;
		wk = lp_wk;*/

		/* don't forget to run no-motion tests */

		/* integrate angular velocity to get the new orientation quaternion */
		double w;
		
		w = sqrt (wi*wi + wj*wj + wk*wk);
		if (w > 0.000001) {
			q0 = cos (w * dt * 0.5);
			q1 = sin (w * dt * 0.5) * wi * (1. / w);
			q2 = sin (w * dt * 0.5) * wj * (1. / w);
			q3 = sin (w * dt * 0.5) * wk * (1. / w);

			s0 = _s0*q0 - _s1*q1 - _s2*q2 - _s3*q3;
			s1 = _s1*q0 + _s0*q1 - _s3*q2 + _s2*q3;
			s2 = _s2*q0 + _s3*q1 + _s0*q2 - _s1*q3;
			s3 = _s3*q0 - _s2*q1 + _s1*q2 + _s0*q3;
		}

		/*s0 = _s0 + 0.5 * dt * (- (_s1 * wi) - (_s2 * wj) - (_s3 * wk));
		s1 = _s1 + 0.5 * dt * (+ (_s0 * wi) - (_s3 * wj) + (_s2 * wk));
		s2 = _s2 + 0.5 * dt * (+ (_s3 * wi) + (_s0 * wj) - (_s1 * wk));
		s3 = _s3 + 0.5 * dt * (- (_s2 * wi) + (_s1 * wj) + (_s0 * wk));

		double s = sqrt (s0*s0 + s1*s1 + s2*s2 + s3*s3);
		s0 = s0 / s;
		s1 = s1 / s;
		s2 = s2 / s;
		s3 = s3 / s;*/

		/* convert acceleration from local coordinates to global;
		   augmented acceleration is (0, ai, aj, ak);
		   conjugate of s is (s0, -s1, -s2, -s3);
		   a_xyz = s * a_ijk * s_conj */
		q0 = 0.*s0 - ai*(-s1) - aj*(-s2) - ak*(-s3);
		q1 = ai*s0 + 0.*(-s1) - ak*(-s2) + aj*(-s3);
		q2 = aj*s0 + ak*(-s1) + 0.*(-s2) - ai*(-s3);
		q3 = ak*s0 - aj*(-s1) + ai*(-s2) + 0.*(-s3);

		ax = s1*q0 + s0*q1 - s3*q2 + s2*q3;
		ay = s2*q0 + s3*q1 + s0*q2 - s1*q3;
		az = s3*q0 - s2*q1 + s1*q2 + s0*q3;

		az -= av_g;

		/* TODO: place mechanical noise filter here */
		/* some sort of: */
		if (fabs(ax) < 0.06)
			ax = 0.;
		if (fabs(ay) < 0.06)
			ay = 0.;
		if (fabs(az) < 0.06)
			az = 0.;

		/* integrate acceleration one time to get velocity 
		   in absolute coordinates */
		vx = _vx + (ax * dt);
		vy = _vy + (ay * dt);
		vz = _vz + (az * dt);

		/* correct velocity
		   At this point our assuptions about plane movement and
		   hard link between orientation and velocity are take place. 
		   ----------------------------------------------------------------- */
		double v;
		v = sqrt ((vx * vx) + (vy * vy));

		//        if (v < 0.001) v = 0.;

		/* find coordinates of vector i=(1,0,0) in system xyz */
		vx = s1*s1 + s0*s0 - s3*s3 - s2*s2;
		vy = 2. * (s2*s1 + s3*s0);
		/* vz = 2. * (s3*s1 - s2*s0); */

		/* normalize ixy */
		double ixy;
		ixy = sqrt ((vx * vx) + (vy * vy));
		vx = v * vx / ixy;
		vy = v * vy / ixy;
		/* ---------------------------------------------------------------------
		   end of assumptions-depended code */

		/* integrate acceleration two times to get absolute coordinates */
		rx  = _rx + _vx * dt + 0.5 * ax * dt * dt;
		ry  = _ry + _vy * dt + 0.5 * ay * dt * dt;
		rz  = _rz + _vz * dt + 0.5 * az * dt * dt;

		/* prepare variables for the next iteration */
		_rx = rx;
		_ry = ry;
		_rz = rz;

		_vx = vx;
		_vy = vy;
		_vz = vz;

		_s0 = s0;
		_s1 = s1;
		_s2 = s2;
		_s3 = s3;

		fprintf (fout, "%lf  ", (double)smpl_cnt / SAMPLING_FREQUENCY);
		fprintf (fout, "%lf %lf %lf  ", rx, ry, rz);
		fprintf (fout, "%lf %lf %lf  ", vx, vy, vz);
		fprintf (fout, "%lf %lf %lf  ", ax, ay, az);
		fprintf (fout, "%lf %lf %lf  ", ai, aj, ak);
		fprintf (fout, "%lf %lf %lf  ", wi, wj, wk);
		fprintf (fout, "\n");

		smpl_cnt++;
	}

	fprintf (stdout, "----\ncount: %ld\n", smpl_cnt);
	fclose (fout);

	return 0;
}

/* callback interface */

static void *callback_data = 0;
static pfn_obtain_sample callback_obtain_sample = 0;

static inline int
obtain_sample (int16_t *ax, int16_t *ay, int16_t *az,
		int16_t *wx, int16_t *wy, int16_t *wz)
{
	return callback_obtain_sample (ax, ay, az, wx, wy, wz, callback_data);
}

void
engine_set_callback_functions (pfn_obtain_sample _obtain_sample, void *_data)
{
	callback_obtain_sample = _obtain_sample;
	callback_data = _data;
}

void
engine_set_calculation_params (double _calibration_time)
{
	calibration_time = _calibration_time;
}


