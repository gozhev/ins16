#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/time.h>
#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>

#include <math.h>
#include "sample.h"

#define CHRDEV "/dev/mpu6050"
#define RAW_DATA_CHUNK_SIZE 22 /* bytes */

#define SAMPLING_FREQUENCY 1000 /* Hz */

#define PHYS_G 9.80665 /* acceleration of gravity in meters per second squared */
/* Full Scale Ranges of Sensors */
#define ACCEL_FS 2 /* acceleration(s) of gravity */
#define GYRO_FS 500.0 /* degrees per sec */
/* software low-pass filter frame size */
#define SW_LOWPASS_SMPLCNT 400 /* samples */

/* convert raw values to SI */
#define A_RAW_TO_SI(a) ( PHYS_G * ACCEL_FS * ((double)a / 32768.0) )
#define W_RAW_TO_SI(w) ( (M_PI * GYRO_FS / 180.0) * ((double)w / 32768.0) )  

long long timediff (const struct timeval *, const struct timeval *);
long get_msecs_of_day (void);
int calculate (const char *);
int collect_samples (const char *, int);
void print_usage (FILE *, char *);
void raw_to_sample (struct sample_t *, uint8_t *);
inline void print_sample (FILE *, struct sample_t *);

int collect_mode;
double calibration_time;

int main (int argc, char **argv)
{
    int rc;
    int opt;
    
    char *fname;
    int seconds_to_collect;
    struct sample_t sample;
    uint8_t buffer [RAW_DATA_CHUNK_SIZE];
    int fd_in;
    size_t smpl_cnt;

    fname = NULL;
    collect_mode = 0;
    calibration_time = 0;

    opterr = 0;
    while ((opt = getopt (argc, argv, "a:f:s:h")) != -1) {
        switch (opt) {
            case 'h': print_usage (stdout, argv[0]);
                return 0;

            case 's':
                rc = sscanf (optarg, "%d", &seconds_to_collect);
                if (rc < 1) {
                    fprintf (stderr, "Failed to parse time parameter.\n");
                    return 1;
                }
                collect_mode = 1;
                break;

            case 'a':
                rc = sscanf (optarg, "%lf", &calibration_time);
                if (rc < 1) {
                    fprintf (stderr, "Failed to parse calibration time parameter.\n");
                    return 1;
                }
                break;

            case 'f': fname = optarg;
                break;

            case '?':
                if (isprint(optopt))
                    fprintf (stderr, "Unknown option or missing option argument: "
                            "`-%c'.\n", optopt);
                else
                    fprintf (stderr, "Unknown option character `\\x%x'.\n", optopt);
                return 1;

            default:
                fprintf (stderr, "Failed to parse options.\n");
                return 1;
        }
    }

    if (collect_mode) {
        if (fname == NULL) {
            fprintf (stderr, "File name omitted.\n");
            return 1;
        }
        
        collect_samples (fname, seconds_to_collect);
        
        return 0;
    }

    if (fname == NULL) {
        fprintf (stderr, "File name omitted. Print-only mode.\n");
        
        fd_in = open (CHRDEV, O_RDONLY);
        if (fd_in < 0) {
            fprintf (stderr, "Failed to open %s for reading.\n", CHRDEV);
            return -errno;
        }
        
        while (1) { /* yes, it wait you to press Ctrl+C */
            rc = read (fd_in, buffer, RAW_DATA_CHUNK_SIZE);
            raw_to_sample (&sample, buffer);
            print_sample (stdout, &sample);     
        }
            
        close (fd_in); 
        return 0;
    }

    calculate (fname);
    
    return 0;
}

int calculate (const char *fname)
{
    int rc;
    int fd_in;
    FILE *fout;
    
    int smpl_cnt, clbr_cnt;
    struct sample_t sample;

    fd_in = open (fname, O_RDONLY);
    if (fd_in < 0) {
        fprintf (stderr, "Failed to open %s for reading.\n", fname);
        return -errno;
    }

    fout = fopen ("out.dat", "w");
    if (fout == 0) {
        fprintf (stderr, "Failed to open %s for writing.\n", "out.dat");
        close (fd_in);
        return -errno;
    }
    
    /* local axes: i, j, k,
        global: x, y, z 
        
        a - acceleration
        w - angular velocity
        v - linear velocity
        r - position vector
        s - orientation quaternion
        
        *x - projection to x
        zo_* - zero-point offset
        av_* - average value
        *_ - previous value
        *0..*3 - quaternion coordinates */
    double ai, aj, ak;
    double wi, wj, wk;
    
    /* software low-pass filter */
    double lp_Ai[SW_LOWPASS_SMPLCNT] = {0.};
    double lp_Aj[SW_LOWPASS_SMPLCNT] = {0.};
    double lp_Ak[SW_LOWPASS_SMPLCNT] = {0.};
    int lp_pos = 0;
    double lp_ai = 0.;
    double lp_aj = 0.;
    double lp_ak = 0.;
    
    double ax, ay, az;
    
    double vx, vy, vz;
    double vx_, vy_, vz_;
    
    double rx, ry, rz;
    double rx_, ry_, rz_;

    /* orientation quaternion:
        s0 = Cos (PHY/2)
        s1 = u * Sin (PHY/2)
        s2 = u * Sin (PHY/2)
        s3 = u * Sin (PHY/2)
        
        where u is a three-dimensional unit vector,
        and PHY is a rotation angle
        
        s0..3 represents rotation around the axis u of angle PHY */
    double s0, s1, s2, s3;
    double s0_, s1_, s2_, s3_;

    double q0, q1, q2, q3;
    
    double dt;
   
    double av_ai, av_aj, av_ak;
    double zo_ai, zo_aj, zo_ak;
    double zo_wi, zo_wj, zo_wk;

    av_ai = av_aj = av_ak = 0.;
    zo_wi = zo_wj = zo_wk = 0.;
    
    /* hardware zero offset
        don't ask me how i get this, it's for debugging only */
    zo_ai = 0.4528;
    zo_aj = -0.14464;
    zo_ak = 0.;

    smpl_cnt = 0;
    clbr_cnt = calibration_time * SAMPLING_FREQUENCY;
    
    /* calibration loop */
    while (smpl_cnt < clbr_cnt) {
        rc = read (fd_in, &sample, sizeof (struct sample_t));
        if (rc == 0)
            break;
        if (rc < sizeof (struct sample_t)) {
            fprintf (stderr, "Corrupted input file. Calculation not finished.\n");
            return -1;
        }
    
        ai = A_RAW_TO_SI (sample.ax);
        aj = A_RAW_TO_SI (sample.ay);
        ak = A_RAW_TO_SI (sample.az);
        wi = W_RAW_TO_SI (sample.gx);
        wj = W_RAW_TO_SI (sample.gy);
        wk = W_RAW_TO_SI (sample.gz);
       
        /* TODO: add averaging filter that uniforms dispersion, not average */

        /* TODO: 
            calibration steps nedded:
            - zero offset for accel
            - sensitivity scaling for accel
            - zero offset for gyro */

        /* I think we can not calibrate zero-point offset of accelerometer
            in-place in static position. Do do so we need to rotate sensor
            180 degrees around the axe which is orthogonal to acceleration
            of gravity during the calibration process. */

        av_ai += ai;
        av_aj += aj;
        av_ak += ak;
        zo_wi += wi;
        zo_wj += wj;
        zo_wk += wk;

        smpl_cnt++;
    }

    av_ai /= smpl_cnt;
    av_aj /= smpl_cnt;
    av_ak /= smpl_cnt;
    zo_wi /= smpl_cnt;
    zo_wj /= smpl_cnt;
    zo_wk /= smpl_cnt;

    av_ai -= zo_ai;
    av_aj -= zo_aj;
    av_ak -= zo_ak;
    
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
    double av_g;

    /* in the theory this should be equal to acceleration of gravity */
    av_g = sqrt ((av_ai * av_ai) + (av_aj * av_aj) + (av_ak * av_ak));
    av_ai /= av_g;
    av_aj /= av_g;
    av_ak /= av_g;

    /* Identity quaternion.
        this is default value, it only true if our sensor is set absolutly
        horizontally */
    s0_ = 1.;
    s1_ = s2_ = s3_ = 0.;

    /* let's find a quaternion corresponding to our real system
        k has the coordinates 0, 0, 1 in the local system,
        z has the coordinates zi, zj, zk in the local system.
        To get transformation quaternion from ijk to xyz we need to
        construct rotation quaternion from z to k.
        NOTE: See ref/math directory for more details. */
    s0_ = sqrt (2. + (2. * av_ak));
    s1_ = av_aj / s0_;
    s2_ = -av_ai / s0_;
    s3_ = 0.;
    s0_ *= 0.5;

    vx_ = vy_ = vz_ = 0.;
    rx_ = ry_ = rz_ = 0.;

    dt = 1. / SAMPLING_FREQUENCY;

    while (1) {
        rc = read (fd_in, &sample, sizeof (struct sample_t));
        if (rc == 0)
            break;
        if (rc < sizeof (struct sample_t)) {
            fprintf (stderr, "Corrupted input file. Calculation not finished.\n");
            return -1;
        }

        /* print_sample (stdout, &sample); */

        ai = A_RAW_TO_SI (sample.ax);
        aj = A_RAW_TO_SI (sample.ay);
        ak = A_RAW_TO_SI (sample.az);
        wi = W_RAW_TO_SI (sample.gx);
        wj = W_RAW_TO_SI (sample.gy);
        wk = W_RAW_TO_SI (sample.gz);

        /* apply zero offset */
        wi -= zo_wi;
        wj -= zo_wj;
        wk -= zo_wk;
        ai -= zo_ai;
        aj -= zo_aj;
        ak -= zo_ak;

        /* software low-pass filter */
        ai /= SW_LOWPASS_SMPLCNT;
        aj /= SW_LOWPASS_SMPLCNT;
        ak /= SW_LOWPASS_SMPLCNT;
        lp_ai = lp_ai - lp_Ai[lp_pos] + ai;
        lp_aj = lp_aj - lp_Aj[lp_pos] + aj;
        lp_ak = lp_ak - lp_Ak[lp_pos] + ak;
        lp_Ai[lp_pos] = ai;
        lp_Aj[lp_pos] = aj;
        lp_Ak[lp_pos] = ak;
        lp_pos++;
        if (lp_pos == SW_LOWPASS_SMPLCNT) lp_pos = 0;
        ai = lp_ai;
        aj = lp_aj;
        ak = lp_ak;

        /* don't forget to run no-motion tests */
        
        /* integrate angular velocity to get the new orientation quaternion */
        double w;

        w = sqrt (wi*wi + wj*wj + wk*wk);
        q0 = cos (w * dt * 0.5);
        q1 = sin (w * dt * 0.5) * wi * (1. / w);
        q2 = sin (w * dt * 0.5) * wj * (1. / w);
        q3 = sin (w * dt * 0.5) * wk * (1. / w);

        s0 = s0_*q0 - s1_*q1 - s2_*q2 - s3_*q3;
        s1 = s1_*q0 + s0_*q1 - s3_*q2 + s2_*q3;
        s2 = s2_*q0 + s3_*q1 + s0_*q2 - s1_*q3;
        s3 = s3_*q0 - s2_*q1 + s1_*q2 + s0_*q3;

        /*s0 = s0_ + 0.5 * dt * (- (s1_ * wi) - (s2_ * wj) - (s3_ * wk));
        s1 = s1_ + 0.5 * dt * (+ (s0_ * wi) - (s3_ * wj) + (s2_ * wk));
        s2 = s2_ + 0.5 * dt * (+ (s3_ * wi) + (s0_ * wj) - (s1_ * wk));
        s3 = s3_ + 0.5 * dt * (- (s2_ * wi) + (s1_ * wj) + (s0_ * wk));

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

        /* TODO: place mechanical noise filter here */
        /* some sort of: */
/*        if (fabs(ax) < 0.08)
            ax = 0.;
        if (fabs(ay) < 0.08)
            ay = 0.;
        if (fabs(az) < 0.08)
            az = 0.;
*/
        /* integrate acceleration one time to get velocity 
            in absolute coordinates */
        vx = vx_ + (ax * dt);
        vy = vy_ + (ay * dt);
        vz = vz_ + (az * dt);
        
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
        vz = 2. * (s3*s1 - s2*s0);

        /* normalize ixy */
        double ixy;
        ixy = sqrt ((vx * vx) + (vy * vy));
        vx = v * vx / ixy;
        vy = v * vy / ixy;
        /* ---------------------------------------------------------------------
            end of assumptions-depended code */
      
        /* integrate acceleration two times to get absolute coordinates */
        rx  = rx_ + vx_ * dt + 0.5 * ax * dt * dt;
        ry  = ry_ + vy_ * dt + 0.5 * ay * dt * dt;
        rz  = rz_ + vz_ * dt + 0.5 * az * dt * dt;

        /* prepare variables for the next iteration */
        rx_ = rx;
        ry_ = ry;
        rz_ = rz;

        vx_ = vx;
        vy_ = vy;
        vz_ = vz;

        s0_ = s0;
        s1_ = s1;
        s2_ = s2;
        s3_ = s3;

        fprintf (fout, "%lf  %lf %lf %lf  %lf %lf %lf  %lf %lf %lf\n", 
                (double)smpl_cnt / SAMPLING_FREQUENCY,
                rx, ry, rz,
                vx, vy, vz,
                ax, ay, az);
        
        smpl_cnt++;
    }
    
    fprintf (stdout, "----\ncount: %ld\n", smpl_cnt);

    fclose (fout);
    close (fd_in);

    return 0;
}

int collect_samples (const char *fname, int seconds)
{
    int rc;
    struct sample_t *samples;
    size_t smpl_sz, smpl_cnt;
    uint8_t buffer [RAW_DATA_CHUNK_SIZE];

    struct timeval s,t,s0;
    int fd_in, fd_out;
    size_t i;

    fd_in = open (CHRDEV, O_RDONLY);
   
    if (fd_in < 0) {
        fprintf (stderr, "Failed to open %s for reading.\n", CHRDEV);
        return -errno;
    }
    
    fd_out = open (fname, O_WRONLY | O_CREAT, 0666);
   
    if (fd_out < 0) {
        fprintf (stderr, "Failed to open %s for writing.\n", fname);
        return -errno;
    }

    smpl_sz = SAMPLING_FREQUENCY * seconds * 1.5;
    samples = malloc (sizeof (struct sample_t) * smpl_sz);

    if (samples == NULL) {
        fprintf (stderr, "Failed to allocate memory for %llu samples.\n", 
                smpl_sz);
        return -errno;
    }
    
    smpl_cnt = 0;

    fprintf (stdout, "Start collecting.\n");
    
    gettimeofday (&s, 0);
    s0 = t = s;
    s.tv_sec += seconds;

    while ((t.tv_sec < s.tv_sec) || (t.tv_usec < s.tv_usec)) {
        rc = read (fd_in, buffer, RAW_DATA_CHUNK_SIZE);
        raw_to_sample (&samples[smpl_cnt], buffer);
        ++smpl_cnt;
        gettimeofday (&t, 0);
    }
    
    close (fd_in);
    
    fprintf (stdout, "Collecting time: %lld us. Samples count: %ld. Writing to file.\n", 
            timediff (&t, &s0), smpl_cnt);
    
    for (i = 0; i < smpl_cnt; ++i) {
        rc = write (fd_out, samples + i, sizeof (struct sample_t));
    }

    close (fd_out);
    free (samples);
    
    fprintf (stdout, "Done.\n");

    return 0;
}

long get_msecs_of_day (void)
{
    struct timeval t;
    gettimeofday (&t, 0);
    return t.tv_sec*1000 + t.tv_usec/1000;
}

long long timediff (const struct timeval *t, const struct timeval *s)
{
    time_t ds = t->tv_sec - s->tv_sec - 1;
    suseconds_t dus = 1000000 - s->tv_usec + t->tv_usec;

    return ds*1000000 + dus;
}

void print_usage (FILE *fout, char *this_name)
{
    fprintf (fout, "Usage:\n"
            "%s [ -a <average seconds> ] "
            "[ -f <file> ] "
            "[ -s <sampling seconds>] "
            "[ -h ]\n", this_name);
    fprintf (fout,
            "Run on board:\n"
            "$ %s -f test001.bin -s 30\n"
            "\tThis creates test001.bin file and fill it with\n"
            "\t30 seconds of samples.\n", this_name);
    fprintf (fout,
            "Run on host:\n"
            "$ %s -f test001.bin -a 10\n"
            "\tThis reads test001.bin, calculates trajectory and\n"
            "\tput coordinates to out.dat file. First 10 seconds\n"
            "\tof samples are used for calibration and not\n"
            "\tappears in out.dat lines.\n", this_name);
}

void raw_to_sample (struct sample_t *sample, uint8_t *raw)
{
    sample->ax = MAKE16 (raw[ 0], raw[ 1]);
    sample->ay = MAKE16 (raw[ 2], raw[ 3]);
    sample->az = MAKE16 (raw[ 4], raw[ 5]);
    sample->gx = MAKE16 (raw[ 8], raw[ 9]);
    sample->gy = MAKE16 (raw[10], raw[11]);
    sample->gz = MAKE16 (raw[12], raw[13]);
    sample->tstmp = *(int64_t *)(raw+14);
}

inline void print_sample (FILE *fout, struct sample_t *sample)
{
    fprintf (fout, "%16lld %6d %6d %6d %6d %6d %6d\n", sample->tstmp,
            sample->ax, sample->ay, sample->az,
            sample->gx, sample->gy, sample->gz);
}
