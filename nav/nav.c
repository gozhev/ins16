#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/time.h>
#include <stdint.h>

#include "sample.h"

#define CHRDEV "/dev/mpu6050"

long get_msecs_of_day (void)
{
    struct timeval t;
    gettimeofday (&t, 0);
    return t.tv_sec*1000 + t.tv_usec/1000;
}

int main (void)
{
    int rc = 0;
    int fd;

    fd = open (CHRDEV, O_RDONLY);
   
    if (fd < 0) {
        fprintf (stderr, "Failed to open %s.\n", CHRDEV);
        return -errno;
    }
    
    struct sample_t sample;
    struct sample_t samples [2000];
    uint8_t buffer [22];
    int count;
    struct timeval s,t;

    gettimeofday (&s, 0);
    t = s;
    s.tv_sec += 1;

    //while ((t.tv_sec < s.tv_sec) || (t.tv_usec < s.tv_usec)) {
    count = 0;
    while (1) {
        rc = read (fd, buffer, 22);
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
       
        //++count;
        //gettimeofday (&t, 0);
    }
    
    close (fd);
    return rc;
}

