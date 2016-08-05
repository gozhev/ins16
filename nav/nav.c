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
#include "engine.h"

#define CHRDEV "/dev/mpu6050"
#define RAW_DATA_CHUNK_SIZE 22 /* bytes */

long long timediff (const struct timeval *, const struct timeval *);
long get_msecs_of_day (void);
int calculate (const char *);
int collect_samples (const char *, int);
void print_usage (FILE *, char *);
void raw_to_sample (struct sample_t *, uint8_t *);
inline void print_sample (FILE *, struct sample_t *);

struct io_data {
	int fd_in;
};

int read_sample (int16_t *, int16_t *, int16_t *,
		int16_t *, int16_t *, int16_t *, void *);

int collect_mode;
double calibration_time;

int main (int argc, char **argv)
{
	int rc;
	int opt;

	int fd_in;
	char *fname;
	int seconds_to_collect;
	struct sample_t sample;
	uint8_t buffer [RAW_DATA_CHUNK_SIZE];
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
	
	struct io_data data;
	data.fd_in = open (fname, O_RDONLY);
	if (data.fd_in < 0) {
		fprintf (stderr, "Failed to open %s for reading.\n", fname);
		return -errno;
	}
	
	engine_set_callback_functions (read_sample, &data);
	engine_set_calculation_params (calibration_time);
	engine_calculate ();

	close (data.fd_in);

	return 0;
}

int read_sample (int16_t *ax, int16_t *ay, int16_t *az,
		int16_t *wx, int16_t *wy, int16_t *wz, void *data)
{
	int rc;
	struct sample_t sample;

	rc = read (((struct io_data *)data)->fd_in,
		&sample, sizeof (struct sample_t));
	if (rc == 0)
		return 1;
	if (rc < sizeof (struct sample_t)) {
		fprintf (stderr, "Corrupted input file. Calculation not finished.\n");
		return -1;
	}
	
	*ax = sample.ax;
	*ay = sample.ay;
	*az = sample.az;
	*wx = sample.wx;
	*wy = sample.wy;
	*wz = sample.wz;

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
	sample->wx = MAKE16 (raw[ 8], raw[ 9]);
	sample->wy = MAKE16 (raw[10], raw[11]);
	sample->wz = MAKE16 (raw[12], raw[13]);
	sample->tstmp = *(int64_t *)(raw+14);
}

inline void print_sample (FILE *fout, struct sample_t *sample)
{
	fprintf (fout, "%16lld %6d %6d %6d %6d %6d %6d\n", sample->tstmp,
		 sample->ax, sample->ay, sample->az,
		 sample->wx, sample->wy, sample->wz);
}
