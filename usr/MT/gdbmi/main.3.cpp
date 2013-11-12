
#include <stdio.h>
#include <stdlib.h>

#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <termios.h>
#include <pty.h>

#include <MT/util.h>

/* Spawns a process with redirected standard input and output
   streams. ARGV is the set of arguments for the process,
   terminated by a NULL element. The first element of ARGV
   should be the command to invoke the process.
   Returns a file descriptor that can be used to communicate
   with the process. */
int main(int argc, char *argv[]) {
  int ret_fd = -1;

  cout <<"1" <<endl;

  /* Find out if the intended programme really exists and
     is accessible. */
  struct stat stat_buf;
  if (stat (argv[0], &stat_buf) != 0) {
    perror ("ERROR accessing programme");
    return -1;
  }

  cout <<"2" <<endl;

  /* Save the standard error stream. */
  int saved_stderr = dup (STDERR_FILENO);
  if (saved_stderr < 0) {
    perror ("ERROR saving old STDERR");
    return -1;
  }

  cout <<"3" <<endl;

  /* Create a pseudo terminal and fork a process attached
     to it. */
  char slaveName[255];
  pid_t pid = forkpty (&ret_fd, slaveName, NULL, NULL);
  if(!pid){
    /* Inside the child process. */

  cout <<"4" <<endl;

    /* Ensure that terminal echo is switched off so that we
       do not get back from the spawned process the same
       messages that we have sent it. */
    struct termios orig_termios;
    if (tcgetattr (STDIN_FILENO, &orig_termios) < 0) {
      perror ("ERROR getting current terminal's attributes");
      return -1;
    }

  cout <<"5" <<endl;

    orig_termios.c_lflag &= ~(ECHO | ECHOE | ECHOK | ECHONL);
    orig_termios.c_oflag &= ~(ONLCR);

  cout <<"6" <<endl;


    if (tcsetattr (STDIN_FILENO, TCSANOW, &orig_termios) < 0) {
      perror ("ERROR setting current terminal's attributes");
      return -1;
    }

  cout <<"7" <<endl;

    /* Restore stderr. */
    if (dup2 (saved_stderr, STDERR_FILENO) < 0) {
      perror ("ERROR restoring STDERR");
      return -1;
    }

  cout <<"8" <<endl;

    /* Now spawn the intended programme. */
    if (execvp (argv[0], argv)) {
      /* execv() should not return. */
      perror ("ERROR spawning programme");
      return -1;
    }
  } else if (pid < 0) {

    std::cout <<"9" <<std::endl;

    perror ("ERROR spawning programme");
    return -1;
  } else {
    std::cout <<"0" <<std::endl;

    MT::wait(1.);
    close (saved_stderr);
  }

  return ret_fd;
}
