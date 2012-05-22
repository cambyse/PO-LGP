#include "biros_threadless.h"

#ifdef BIROS_THREADLESS

#include <stdlib.h>
#include <stdio.h>
#include <biros/biros_internal.h>
#include <dirent.h>
#include <errno.h>

void loopSerialized(const ProcessL& P) {
  Threadless threadless;
  cout << "starting serialization" << endl;
  while (true) {
    int pID = threadless.nextProcess();
    //cout << "next step by " << pID << endl;
    // job done
    if (-1 == pID) {
      cout << "serialization done" << endl;
      break;
    }
    if (-2 == pID) {
      cout << "serialization not possible" << endl;
      break;
    }
    Process* p;
    uint i;
    for_list(i, p, P) {
      //cout << "checking " << p->id << endl;
      if ((int) p->id == pID) {
        cout << "step by process " << pID << endl;
        if (p->s->threadCondition.state==tsCLOSE) p->threadOpen();
        while (!p->threadIsIdle()) sleep(0.000001);
        p->threadStep();
      }
    }
  }
}

struct Log {
  bool isWrite;
  int pID;
  int step;
};

struct Action {
  int variable, log_index;
};

struct sThreadless {
  MT::Array<MT::Array<Log> > logs; // list of log entries for every variable
  
  // get all filenames in a directory
  int getdir(MT::Array<MT::String> &files, const MT::String dir) {
    DIR *dp;
    struct dirent *dirp;
    if ((dp  = opendir(STRING(dir))) == NULL) {
      std::cout << "Error(" << errno << ") opening " << dir << std::endl;
      return errno;
    }
    
    while ((dirp = readdir(dp)) != NULL) {
      files.append(MT::String(dirp->d_name));
    }
    closedir(dp);
    return 0;
  }
  
  // read variable log from filename
  void readfile(MT::String& filename) {
    //cout << "reading file " << logs.N << ": " << filename << endl;
    MT::Array<Log> *var_log = new MT::Array<Log>;
    var_log->memMove = true;
    logs.append(*var_log);
    FILE* accessInputFile = fopen(STRING("log/" << filename), "r");
    CHECK(accessInputFile, "can't open file " << filename);
    char readOrWrite;
    uint revision;
    int pid, step_count;
    while (EOF != fscanf(accessInputFile, "%c,%u,%d,%d\n", &readOrWrite,
                         &revision, &pid, &step_count)) {
      if (pid > -1 && step_count > -1) {
        Log *log = new Log();
        log->isWrite = readOrWrite == 'w';
        log->pID = pid;
        log->step = step_count;
        logs.last().append(*log);
      }
    }
  }
};

// get logs from files
Threadless::Threadless() :
    s(new sThreadless) {
  MT::Array<MT::String> filenames;
  s->getdir(filenames, MT::String("./log/"));
  MT::String name;
  for_list_(name, filenames) {
    if ('a' == name(0)) {
      s->readfile(name);
    }
  }
}

// returns pID of next process
// -1 if serialisation is done
// -2 if serialisation is not possible
const int Threadless::nextProcess() {
  // 1.
  // find number of possible accesses from log which are allowed
  // at every variable now
  uint numVars = s->logs.N;
  uintA numPossibleAccesses;
  numPossibleAccesses.resize(numVars);
  numPossibleAccesses.setZero();
  for (uint var=0; var < numVars; var++) {
    for (uint i=0; i < s->logs(var).N; i++) {
      Log access = s->logs(var)(i);
      numPossibleAccesses(var)++;
      // if there are more accesses in this var log
      if (i+1 < s->logs(var).N) {
        Log nextAccess = s->logs(var)(i+1);
        // if the next access is part of the same process step
        if (nextAccess.pID == access.pID && nextAccess.step == access.step) {
          continue;
          // if current or next access is write access
        } else if (access.isWrite || nextAccess.isWrite) {
          break;
        }
      }
    }
  }
  
  //cout << "testing for conflicts:" << endl;
  
  // 2.
  // test for conflicts
  Log access;
  MT::Array<Action> allowed;
  bool conflict = true;
  // go through all possible accesses
  for (uint var=0; var < numVars; var++) {
    for (uint i=0; i < numPossibleAccesses(var); i++) {
      access = s->logs(var)(i);
      //cout << "testing: pID " << access.pID << ", step " << access.step << endl;
      allowed.clear();
      conflict = false;
      // test for conflicts for this access in all variable logs
      for (uint var2=0; var2 < numVars; var2++) {
        MT::Array<Log> log = s->logs(var2);
        // go through log
        for (uint j=0; j < log.N; j++) {
          // same pID
          if (log(j).pID == access.pID) {
            // no conflict in this log found
            if (log(j).step > access.step) {
              //cout << "-> no conflict, variable " << var2 << endl;
              break;
              // process conflict: other steps have to be done before
            } else if (log(j).step < access.step) {
              //cout << "-> process conflict, variable " << var2 << endl;
              conflict = true;
              break;
              // variable conflict: other reads/writes have to be done before
            } else {
              if (j >= numPossibleAccesses(var2)) {
                //cout << "-> variable conflict, variable" << var2 << endl;
                conflict = true;
                break;
              } else {
                //cout << "-- allowed access, variable" << var2 << endl;
                // memorize allowed accesses
                Action action;
                action.variable = var2;
                action.log_index = j;
                allowed.append(action);
              }
            }
          }
        }
        // if conflict, try next access
        if (conflict) break;
      }
      // if access without conflict found, you're done
      if (!conflict) break;
    }
    // if access without conflict found, you're done
    if (!conflict) break;
  }
  
  // serialization done, because all logs are empty
  if (0 == sum(numPossibleAccesses)) {
    return -1;
    // serialization not possible, because no access without conflict found
  } else if (conflict) {
    return -2;
    // keep working: remove step from logs and return pID
  } else {
    for (int i=allowed.N-1; i>=0; i--) {
      s->logs(allowed(i).variable).remove(allowed(i).log_index);
    }
    return access.pID;
  }
}

#else
#endif
