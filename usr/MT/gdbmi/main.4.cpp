#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <pty.h>
#include <utmp.h>
#include <ctype.h>
#include <errno.h>

#include <MT/util.h>

void pty_send(int master, const char *str){
  write(master, str, strlen(str));
}

int main(int argc, char** argv) {
  char buf[BUFSIZ] = {0};
  int master;
  int ret = forkpty(&master, NULL, NULL, NULL);
  
  if (ret == -1)
    puts("no fork"), exit(0);
    
  if (!ret) {
    printf("I'M HERE\n");
    execlp("gdb", "sh", NULL);
    exit(0);
  }
  
  sleep(1); /* let the shell run */
  
  
  if (argc >= 2) {
    write(master, argv[1], strlen(argv[1]));
    write(master, "\n", 1);
  } else {
    pty_send(master, "help\nquit\n");
  }
  
  
  while (1) {
    switch (ret = read(master, buf, BUFSIZ)) {
      case -1:
        puts("error!HERE");
        
        cout <<strerror(errno) <<'(' <<errno <<')' <<endl;
        exit(1);
        break;
      case 0:
        puts("nothing.."), sleep(1);
        break;
      default:
        buf[ret] = '\0';
        puts(buf);
        
    }
  }
  
  close(master);
  
  return 0;
}
