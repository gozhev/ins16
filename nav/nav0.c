#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/time.h>
#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>

#include <math.h>
#include "sample.h"

#define RAW_DATA_CHUNK_SIZE 22
#define SAMPLING_FREQUENCY 1000
#define CHRDEV "/dev/mpu6050"

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

int calculate (const char *);

int collect_mode;
double calibration_time;

int main (int argc, char **argv)
{
    int rc = 0;
    
    char *fname;

    int seconds;
    int opt;
        
    struct sample_t sample;
    uint8_t buffer [RAW_DATA_CHUNK_SIZE];
    int fd_in;
    size_t smpl_cnt;
    
    fname = 0;
    collect_mode = 0;
    calibration_time = 0;
    opterr = 0;
    while ((opt = getopt (argc, argv, "a:f:s:")) != -1) {
        switch (opt) {
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

        return collect_samples (fname, seconds);
    }
    
    if (fname == NULL) {
        fprintf (stderr, "File name omitted.\n");
        
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
  


    #define PHYS_G 9.80665
    #define LSB_PER_G 8192.0
    
    #define A_RAW_TO_SI(a) ( PHYS_G * (a / LSB_PER_G) )

    
    /* local axes: i, j
        global: x, y */
    double ai, aj, ax, ay;
    double ix, jx, iy, jy;

    double _vx, _vy, vx, vy;
    double _x, _y, x, y;

    double oi, oj;

    double v;

    double dt;
    
    dt = 0.001;

    ix = 1, jx = 0;
    iy = 0, jy = 1;
    
    x = 0;
    y = 0;

    vx = 0;// 0.01;
    vy = 0;
    
    oi = 0;// 0.000311;
    oj = 0; //-0.046656;

    smpl_cnt = 0;

    int calibr_secs = calibration_time * 1000;

    while (1) {
        rc = read (fd_in, &sample, sizeof (struct sample_t));
        if (rc == 0)
            break;
        if (rc < sizeof (struct sample_t)) {
            fprintf (stderr, "Corrupted input file.\n");
            return -1;
        }
        smpl_cnt++;
        
        /*
        fprintf (stdout, "%16lld %6d %6d %6d %6d %6d %6d\n", sample.tstmp,
                    sample.ax, sample.ay, sample.az,
                    sample.gx, sample.gy, sample.gz);*/

        ai = A_RAW_TO_SI (sample.ax);//0.0001;
        aj = A_RAW_TO_SI (sample.ay);//0.010;
        
//        if (sqrt(ai*ai + aj*aj) < 0.13) continue;


        if (smpl_cnt < calibr_secs) {
            oi += ai;
            oj += aj;
            
            continue;
        }
        
        if (smpl_cnt == calibr_secs) {
            oi /= smpl_cnt;
            oj /= smpl_cnt;
        }
        
        ai -= oi;
        aj -= oj;

        ax = (ix * ai) + (jx * aj);
        ay = (iy * ai) + (jy * aj);

        _vx = vx + (ax * dt);
        _vy = vy + (ay * dt);
        
        _x = x + (vx * dt) + (0.5 * ax * dt * dt);
        _y = y + (vy * dt) + (0.5 * ay * dt * dt);

        vx = _vx;
        vy = _vy;

        x = _x;
        y = _y;

        v = sqrt ((vx * vx) + (vy * vy));

        ix = vx / v;
        iy = vy / v;
        jx = -iy;
        jy = ix;


        fprintf (fout, "%lf %lf %lf\n", x, y, v);
    }
    
    fprintf (stdout, "----\ncount: %ld\n", smpl_cnt);
    
    fprintf (stdout, "calibration: oi=%lf, oj=%lf\n", oi, oj);

    fclose (fout);
    close (fd_in);

    return 0;
}









/*
int print_file (const char *fname)
{
    int rc;
    int fd_in;
    struct sample_t smpl0, smpl;
    fd_in = open (fname, O_RDONLY);
    
    if (fd_in < 0) {
        fprintf (stderr, "Failed to open %s for reading.\n", fname);
        return -errno;
    }
   
    smpl_cnt = 0;
    while (1) {
        rc = read (fd_in, &sample, sizeof (struct sample_t));
        if (rc == 0)
            break;
        if (rc < sizeof (struct sample_t)) {
            fprintf (stderr, "Corrupted input file.\n");
            return -1;
        }

        fprintf (stdout, "%16lld %6d %6d %6d %6d %6d %6d\n", sample.tstmp,
                    sample.ax, sample.ay, sample.az,
                    sample.gx, sample.gy, sample.gz);
        ++smpl_cnt;
    }

    fprintf (stdout, "----\ncount: %ld\n", smpl_cnt);

    close (fd_in);

    return 0;
}
*/
