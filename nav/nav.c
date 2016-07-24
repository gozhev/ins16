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

#define RAW_DATA_CHUNK_SIZE 22
#define SAMPLING_FREQUENCY 1000
#define CHRDEV "/dev/mpu6050"

#define PHYS_G 9.80665 /* acceleration of gravity in meters per second squared */
/* Full Scale Ranges of Sensors */
#define ACCEL_FS 2.0 /* acceleration(s) of gravity */
#define GYRO_FS 1000.0 /* degrees per sec */

/* convert raw values to SI */
#define A_RAW_TO_SI(a) ( PHYS_G * ACCEL_FS * ((double)a / 32768.0) )
#define W_RAW_TO_SI(w) ( (M_PI * GYRO_FS / 180.0) * ((double)w / 32768.0) )  

long long timediff (const struct timeval *, const struct timeval *);
long get_msecs_of_day (void);
int calculate (const char *);
int collect_samples (const char *, int);

int collect_mode;
double calibration_time;

int main (int argc, char **argv)
{
    int rc = 0;
    int opt;
    
    char *fname;
    int seconds;
    struct sample_t sample;
    uint8_t buffer [RAW_DATA_CHUNK_SIZE];
    int fd_in;
    size_t smpl_cnt;

    fname = 0;
    collect_mode = 0;
    calibration_time = 0;
    opterr = 0;
    while ((opt = getopt (argc, argv, "a:f:s:h")) != -1) {
        switch (opt) {
            case 'h':
                fprintf (stdout, "Usage:\n"
                        "%s [ -a <average seconds> ] "
                        "[ -f <file> ] "
                        "[ -s <sampling seconds>] "
                        "[ -h ]\n", argv[0]);
                fprintf (stdout,
                        "Run on board:\n"
                        "$ %s -f test001.bin -s 30\n"
                        "\tThis creates test001.bin file and fill it with\n"
                        "\t30 seconds of samples.\n", argv[0]);
                fprintf (stdout,
                        "Run on host:\n"
                        "$ %s -f test001.bin -a 10\n"
                        "\tThis reads test001.bin, calculates trajectory and\n"
                        "\tput coordinates to out.dat file. First 10 seconds\n"
                        "\tof samples are used for calibration and not\n"
                        "\tappears in out.dat lines.\n", argv[0]);
                return 0;

            case 's':
                rc = sscanf (optarg, "%d", &seconds);
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

            case 'f':
                fname = optarg;
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
        
        collect_samples (fname, seconds);
        
        return 0;
    }
   

    if (fname == NULL) {
        fprintf (stderr, "File name omitted. Print-only mode.\n");
        
        fd_in = open (CHRDEV, O_RDONLY);
       
        if (fd_in < 0) {
            fprintf (stderr, "Failed to open %s for reading.\n", CHRDEV);
            return -errno;
        }
        
        while (1) {
            rc = read (fd_in, buffer, RAW_DATA_CHUNK_SIZE);

            sample.ax = MAKE16 (buffer[ 0], buffer[ 1]);
            sample.ay = MAKE16 (buffer[ 2], buffer[ 3]);
            sample.az = MAKE16 (buffer[ 4], buffer[ 5]);
            sample.gx = MAKE16 (buffer[ 8], buffer[ 9]);
            sample.gy = MAKE16 (buffer[10], buffer[11]);
            sample.gz = MAKE16 (buffer[12], buffer[13]);
            sample.tstmp = *(int64_t *)(buffer+14);
           
            fprintf (stdout, "%16lld %6d %6d %6d %6d %6d %6d\n", sample.tstmp,
                    sample.ax, sample.ay, sample.az,
                    sample.gx, sample.gy, sample.gz);
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
    struct sample_t sample;
    size_t smpl_cnt;
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
        
        *i - projection to i
        o* - zero-point offset
        *_ - previous value */

    double ai, aj;
    double wk;
    
    double ai_, aj_;
    double wk_;

    double oai, oaj;
    double owk;

    double x_, y_, x, y;
    double u_, u, u2;
    double v_, v;

    double dt;

    double ax, ay, ax_, ay_;
    double vx, vy, vx_, vy_;
    
    double ix, jx, iy, jy;
    ix = 1, jx = 0;
    iy = 0, jy = 1;

    ax_ = 0;
    ay_ = 0;
    
    vx_ = 0;
    vy_ = 0;
    
    dt = 0.001;

    ai_ = 0;
    aj_ = 0;
    wk_ = 0;

    x_ = 0;
    y_ = 0;
    u_ = 0;
    v_ = 0;

    oai = 0;
    oaj = 0;
    owk = 0;

    smpl_cnt = 0;

    int clbr_cnt = calibration_time * 1000;


    while (1) {
        rc = read (fd_in, &sample, sizeof (struct sample_t));
        if (rc == 0)
            break;
        if (rc < sizeof (struct sample_t)) {
            fprintf (stderr, "Corrupted input file.\n");
            return -1;
        }
        smpl_cnt++;

        ai = A_RAW_TO_SI (sample.ax);
        aj = A_RAW_TO_SI (sample.ay);
        wk = W_RAW_TO_SI (sample.gz);
        
        if (smpl_cnt < clbr_cnt) {
            oai += ai;
            oaj += aj;
            
            owk += wk;
            
            continue;
        }
        
        if (smpl_cnt == clbr_cnt) {
            oai /= smpl_cnt;
            oaj /= smpl_cnt;

            owk /= smpl_cnt;
        }
        
        ai -= oai;
        aj -= oaj;
        
        wk -= owk;

/*        if (sqrt(ai*ai + aj*aj) < 0.1) {
            ai = 0;
            aj = 0;
        };

        if (fabs(wk) < 0.2) {
            wk = 0;
        };
*/
        
        /* prediction */
/*        u  = u_;
        if (fabs(v_) > 0.0001)
            u +=    ((aj_ / v_) * dt);
        u2 = u_ +          (wk_ * dt);*/
//        v  = v_ +          (ai_ * dt);
        
        u = u_ + (wk * dt);

        /* correction */
        ax = ai * cos (u) - aj * sin (u);
        ay = ai * sin (u) + aj * cos (u);
        //ax = (ix * ai) + (jx * aj);
        //ay = (iy * ai) + (jy * aj);

        vx = vx_ + (ax * dt);
        vy = vy_ + (ay * dt);
        
//        v  = v_ + (0.5 * (ai_ + ai) * dt);
        v = sqrt(vx*vx + vy*vy);

        vx = v * cos (u);
        vy = v * sin (u);
        
/*        u  = u_;
        if (fabs(v_) > 0.0001)
            u += (0.5 * (aj_ / v_) * dt);
        if (fabs(v) > 0.0001)
            u += (0.5 *   (aj / v) * dt);
        u2 = u_ + (0.5 * (wk_ + wk) * dt);
*/        
        
        /* complement filter */
  //      double f = 0;
   //     u = (f * u) + ((1 - f) * u2);
        
        
        x  = x_ + vx_ * dt + 0.5 * ax * dt * dt;
        y  = y_ + vy_ * dt + 0.5 * ay * dt * dt;
        //x  = x_ + (0.5 * ((v_ * cos(u_)) + (v * cos(u))) * dt) + (((ax_ / 3) + (ax / 6)) * dt * dt);
        //y  = y_ + (0.5 * ((v_ * sin(u_)) + (v * sin(u))) * dt) + (((ay_ / 3) + (ay / 6)) * dt * dt);
        
        ai_ = ai;
        aj_ = aj;
        wk_ = wk;

        u_ = u;
        x_ = x;
        y_ = y;
        v_ = v;

        ax_ = ax;
        ay_ = ay;
        vx_ = vx;
        vy_ = vy;

        


        /*vx = vx_ + (ax_ * dt);
        vy = vy_ + (ay_ * dt);

        v = sqrt ((vx * vx) + (vy * vy));

        if (0.0001 < fabs(v)) {
            ix = vx / v;
            iy = vy / v;
            jx = -iy;
            jy = ix;
        }

        ax = (ix * ai) + (jx * aj);
        ay = (iy * ai) + (jy * aj);

        vx = vx_ + (0.5 * (ax_ + ax) * dt);
        vy = vy_ + (0.5 * (ay_ + ay) * dt);

        v = sqrt ((vx * vx) + (vy * vy));

        if (0.0001 < fabs(v)) {
            ix = vx / v;
            iy = vy / v;
            jx = -iy;
            jy = ix;
        }
        
        ax = (ix * ai) + (jx * aj);
        ay = (iy * ai) + (jy * aj);

        x = x_ + (vx_ * dt) + (((ax_ / 3) + (ax / 6)) * dt * dt);
        y = y_ + (vy_ * dt) + (((ay_ / 3) + (ay / 6)) * dt * dt);*/


        fprintf (fout, "%lf %lf %lf\n", x, y, v);
    }
    
    fprintf (stdout, "----\ncount: %ld\n", smpl_cnt);
    
    fprintf (stdout, "calibration: oai=%lf, oaj=%lf\n", oai, oaj);

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
    
    fd_out = open (fname, O_WRONLY | O_CREAT);
   
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

        samples[smpl_cnt].ax = MAKE16 (buffer[ 0], buffer[ 1]);
        samples[smpl_cnt].ay = MAKE16 (buffer[ 2], buffer[ 3]);
        samples[smpl_cnt].az = MAKE16 (buffer[ 4], buffer[ 5]);
        samples[smpl_cnt].gx = MAKE16 (buffer[ 8], buffer[ 9]);
        samples[smpl_cnt].gy = MAKE16 (buffer[10], buffer[11]);
        samples[smpl_cnt].gz = MAKE16 (buffer[12], buffer[13]);
        samples[smpl_cnt].tstmp = *(int64_t *)(buffer+14);
        
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
