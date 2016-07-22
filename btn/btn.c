#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <signal.h>

int fd_btn, fd_yellow;

void child_signal_handler (int signum)
{
    write (fd_yellow, "0", 1);
    close (fd_yellow);
    exit (1);
}

int main (void)
{
    pid_t pid;
    int state;

    system ("/root/dvlp/nav/mknod");

    fd_btn = open ("/sys/class/gpio/export", O_WRONLY);
    write (fd_btn, "23", 2);
    write (fd_btn, "24", 2);
    close (fd_btn);
    
    fd_btn = open ("/sys/class/gpio/gpio23/direction", O_WRONLY);
    write (fd_btn, "out", 3);
    close (fd_btn);
    
    fd_btn = open ("/sys/class/gpio/gpio23/value", O_WRONLY);
    write (fd_btn, "1", 1);
    close (fd_btn);
    
    fd_yellow = open ("/sys/class/gpio/export", O_WRONLY);
    write (fd_yellow, "17", 2);
    close (fd_yellow);
    
    fd_yellow = open ("/sys/class/gpio/gpio17/direction", O_WRONLY);
    write (fd_yellow, "out", 3);
    close (fd_yellow);
    
    fd_yellow = open ("/sys/class/gpio/gpio17/value", O_WRONLY);

    fd_btn = open ("/sys/class/gpio/gpio24/value", O_RDONLY);

    int cnt;
    struct timespec s = {.tv_sec = 0, .tv_nsec=200000000};
    cnt = 0;
    while (1) {
       if (cnt%2)
           write (fd_yellow, "0", 1);
       else 
           write (fd_yellow, "1", 1);
       cnt++;
       if (cnt > 15)
           break;
       nanosleep (&s, NULL); 
    }

    struct timespec t = {.tv_sec = 0, .tv_nsec=10000000};
    char c, c_;

    c_ = '1';
    state = 0;
    while (1) {
        read (fd_btn, &c, 1);
        lseek (fd_btn, 0, SEEK_SET);
        
        if (c == '1' && c_ == '0') {
            state = 1 - state;

            if (state == 1) {
                pid = fork ();
                if (pid == 0) {
                    signal  (SIGINT, child_signal_handler);
                    close (fd_btn);
                    write (fd_yellow, "1", 1);
                    sleep (1);
                    write (fd_yellow, "0", 1);
                    close (fd_yellow);
                    execl ("/root/dvlp/nav/nav", "nav", NULL);
                }
            } else {
                kill (pid, SIGINT);
                waitpid (pid);
            }

        }

        c_ = c;

        nanosleep (&t, 0);
    }

    return 0;
}
