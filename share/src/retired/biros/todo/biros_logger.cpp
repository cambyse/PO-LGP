#include "biros_logger.h"

#ifdef BIROS_LOGGER

#include <stdlib.h>
#include <stdio.h>
#include <biros/biros_internal.h>
//need this for EINVAL and EBUSY pthread_mutex_trylock return values
#include <errno.h>
//need this for directory checking IS_DIR
#include <sys/stat.h>

using std::string;
using std::endl;
using std::ofstream;
using std::ostream;
using std::fstream;
using std::iostream;

Logger logService;

#define MAX(a,b) (a < b? b : a)

//re-add if used, comment if not used (yields "defined but not used" warning)
//static pthread_mutex_t g_coutMutex;

typedef enum en_readAccessConVarStates {
  READ_ACCESS_COND_VAR_STATE_LOCKED = 0,
  READ_ACCESS_COND_VAR_STATE_UNLOCKED = 1,
} readAccessCondVarStates_t;

typedef enum en_writeAccessConVarStates {
  WRITE_ACCESS_COND_VAR_STATE_LOCKED = 0,
  WRITE_ACCESS_COND_VAR_STATE_UNLOCKED = 1,
} writeAccessCondVarStates_t;

struct Revision {
  int write_pid;
  intA read_pids;
  
  /* the amount of processes waiting for this revision */
  uint readsToFinish;
  pthread_mutex_t readsToFinishMutex;
  
  Revision() {
    pthread_mutex_init(&readsToFinishMutex, NULL);
    readsToFinish = 0;
  }
  
  ~Revision() {
  
    pthread_mutex_destroy(&readsToFinishMutex);
  }
};

struct VariableHistory {
  MT::Array<Revision> revisions;
  pthread_mutex_t revisionAccessMutex;
  
  /* here every process sleeps when they want to access a variable not having the correct revision yet */
  ConditionVariable readRevisionCondVar;
  /* here everyone sleeps who wants to have write access */
  ConditionVariable writeRevisionCondVar;
  
  VariableHistory() {
    pthread_mutex_init(&revisionAccessMutex, NULL);
  }
  
  ~VariableHistory() {
  
    pthread_mutex_destroy(&revisionAccessMutex);
  }
};

/**
 * Keeps a managed and synchronised pool of streams.
 *
 * Managed: Handle size of pool, destroys created instances after disposing pool
 * Synchronised: structural changes to internal arrays are synchronised, keeps mutexes for accessing a file.
 *
 * Requires: LOGSTREAM must be either ifstream or ofstream.
 */
template<class LOGSTREAM>
class LogFilePool {
private:

  /*STATIC SECTION */
  
  static bool isDir(string &path) {
    struct stat sCheckPath;
    const int rCheckPath = stat(path.c_str(), &sCheckPath);
    
    // already an existing directory? nothing to do!
    if ((0 == rCheckPath) && S_IFDIR == (sCheckPath.st_mode & S_IFMT)) {
      return true;
    }
    
    return false;
  }
  
  /**
   * Extract path component from full filename path (filename and path)
   *
   * path is guaranteed to end with '/' if return value is true.
   */
  static bool stripFileName(string &path) {
  
    CHECK(NULL != &path, "A path must be given but is '" << &path << "'.");
    
    int index = 0;
    bool found = false;
    for (int pathSize = path.size();
         index<pathSize;
         index++) {
         
      if ('/' == path[index])
        found = true;
        
    }//for
    
    //the string contains no delimters
    if (!found)
      return false;
      
    //reduce to prefix, if no delimiter at the end
    if (index + 1 < (int)path.length())
      path.resize(index+1);
      
    return true;
  }//extractPath
  
  static void makePath(char *fullPath) {
  
    //copy of fullPath
    string cleanPath = fullPath;
    
    //did extraction fail? then fullPath didn't contain a path component. Nothing to do.
    if (!stripFileName(cleanPath))
      return;
    else
      CHECK(0 < cleanPath.size(), "A directory separator was found but the corresponding string is empty.");
      
    //TODO read umask instead of assuming a value
    const int dirmode = 0771;
    
    int pathIndex = 0;
    const int pathSize = cleanPath.size();
    do {
    
      if ('/' != cleanPath[pathIndex])
        continue;
      string dirPart = cleanPath.substr(0, pathIndex);
      
      //found a delimiter but path already exists
      if (isDir(dirPart))
        continue;
        
      int mkresult = 0;
      CHECK_EQ(0 , (mkresult = mkdir(dirPart.c_str(), dirmode)), "mkdir failed with '" << mkresult << "' although '" << dirPart << "' didn't exist.");
      
    } while (++pathIndex < pathSize);
    
  }//makePath
  
  /**
     * Requires file stream, such as ifstream or ofstream. In particular methods open() and is_open() are used.
     */
  static LOGSTREAM& openAndVerify(LOGSTREAM& stream, char *pathName) {
  
    makePath(pathName);
    
    // get the stream ready
    stream.open(pathName);
    
    // verify stream
    CHECK(stream.is_open(),
          "Can't find log file '"
          << pathName
          << "'. Underlying stream complains 'not open'.");
          
    return stream;
    
  }//openAndVerify
  
  /**
   * Reasonable minimum size of an array
   */
  static const int arrayMinSize;
  
  /* OBJECT MEMBERS SECTION */
  /**
   * The actual pool of log stream
   */
  MT::Array<LOGSTREAM*> logFiles;
  
  /**
   * A pool of mutexes: one mutex per log
   */
  MT::Array<pthread_mutex_t*> perLogFileMutex;
  pthread_mutexattr_t perLogFileMutex_attribute;
  
  /**
   * A mutex securing structural changes to the arrays
   */
  pthread_mutex_t arrayMutex;
  
  /* INIT SECTION */
  
  /**
   * Create array item for logFiles and perLogFileMutex and initialise mutex.
   * If permissive the call simply returns on duplicate content instead of complaining/exiting.
   *
   * Returns true if the item was created/initialised. False if the item existed.
   *
   * Requires: index must be an array indexer!
   */
  bool initItem(const int index, const bool permissive) {
  
    //pre conditions
    CHECK(-1 < index, "The parameter index must not be negative, it is an array indexer!");
    
    const uint arrayLowerBound = (index < arrayMinSize) ? arrayMinSize : index+1;
    //use exponential growth of array, but initialise only used items
    const uint arrayUpperBound = arrayLowerBound * arrayLowerBound;
    
    //start structural changes
    pthread_mutex_lock(&arrayMutex);
    
    //first: grow arrays
    while (perLogFileMutex.N < arrayUpperBound)
      perLogFileMutex.append(NULL);
      
    while (logFiles.N < arrayUpperBound)
      logFiles.append(NULL);
      
    if (NULL != perLogFileMutex(index) || NULL != logFiles(index)) {
      pthread_mutex_unlock(&arrayMutex);
      if (permissive)
        return false;
      HALT("You tried to re-initialise the index '"
           << index
           << "'. Either log or mutex where not NULL although required.");
    }
    
    //init mutexes index
    pthread_mutex_t *mutex = new pthread_mutex_t;
    pthread_mutex_init(mutex, &perLogFileMutex_attribute);
    perLogFileMutex(index) = mutex;
    
    //init log index
    LOGSTREAM *log = new LOGSTREAM();
    logFiles(index) = log;
    
    //post conditions
    CHECK(NULL != perLogFileMutex(index),
          "Mutexes array contains NULL although I initialised it on index '"
          << index
          << "'.");
    CHECK(NULL != logFiles(index),
          "Logs array contains NULL on index '"
          << index
          << "' although I've just initialised it.");
          
    //finished structural changes
    pthread_mutex_unlock(&arrayMutex);
    
    return true;
  }//initItem
  
public:

  /* STATIC SECTION */
  
  /* OPERATOR SECTION */
  
  /**
   * Aquires a lock. NO array initialisation! NO stream opening/verification! Use operator()(int, MT::String) to prepare the stream.
   *
   * A previously prepared stream is returned.
   *
   * Remark: The lock must be freed after use using release();
   */
  LOGSTREAM& operator()(const int index) {
  
    //this mutex is of type PTHREAD_MUTEX_ERRORCHECK, hence it will NOT block if owning thread locks again. Function continues and returns error EDEADLK.
    pthread_mutex_lock(perLogFileMutex(index));
    return *logFiles(index);
    
  }//operator() (int)
  
  /**
   * Initialises arrays, aquires a lock, opens and verifies the stream.
   *
   * A ready-to-use stream is returned.
   *
   * Remark: The lock must be freed after use using release();
   */
  LOGSTREAM& operator()(const int index, char* pathname, const bool permissive) {
  
    CHECK(-1 < index, "The parameter index must not be negative, it is an array indexer!");
    
    if (initItem(index, permissive))
      openAndVerify(*logFiles(index), pathname);
      
    return operator()(index);
    
  }//operator() (int, MT::String)
  
  /* INIT SECTION */
  
  LogFilePool() {
  
    pthread_mutexattr_init(&perLogFileMutex_attribute);
    //PTHREAD_MUTEX_ERRORCHECK ==> owning thread may call lock without penalty
    pthread_mutexattr_settype(&perLogFileMutex_attribute, PTHREAD_MUTEX_ERRORCHECK);
    
    pthread_mutex_init(&arrayMutex, NULL);
    
  }//LogFilePool()
  
  ~LogFilePool() {
  
    //starting structural changes
    pthread_mutex_lock(&arrayMutex);
    
    const int logFilesN = logFiles.N;
    const int perLogFilesN = perLogFileMutex.N;
    
    //close and delete all streams
    for (int i=0; i < logFilesN; i++) {
    
      //only destroy the item if no one else owns a lock
      pthread_mutex_t *lock = perLogFileMutex(i);
      if (NULL != lock) {
      
        int tried = 0;
        //while lock is a proper mutex and mutex is already locked
        while (EINVAL != (tried=pthread_mutex_trylock(lock)) && EBUSY == tried) {
          //force other threads out of this lock, even self
          pthread_mutex_unlock(lock);
          cerr  << "WARNING: Kicked thread from lock '"
                << i
                << "' while destroying pool."
                << endl;
        }
        
      }
      
      //although the mutex may be null the log stream might not
      LOGSTREAM *log = logFiles(i);
      if (NULL != log) {
        log->close();
        delete log;
      }
      
      if (NULL != lock) {
        pthread_mutex_unlock(lock);
      }
      
    }//for logFiles
    
    //free array's NULL-items
    logFiles.clear();
    
    //free all mutexes
    for (int i=0; i < perLogFilesN; i++) {
    
      //mutex and attribute are 1-to-1 related (invariant), hence destroy them together
      pthread_mutex_t *lock = perLogFileMutex(i);
      
      if (NULL == lock)
        continue;
        
      pthread_mutex_destroy(lock);
      
      //per log mutexes where created using new, hence a delete is required
      delete lock;
    }//for perLogFileMutex
    
    //free array's NULL-items
    perLogFileMutex.clear();
    
    //finished structural changes
    pthread_mutex_unlock(&arrayMutex);
    
    //pool is disposed
    pthread_mutexattr_destroy(&perLogFileMutex_attribute);
    pthread_mutex_destroy(&arrayMutex);
    
  }//~LogFilePool
  
  /* OBJECT MEMBERS SECTIONS */
  
  /**
   * Release a lock for a specific stream.
   */
  void release(int index) {
  
    pthread_mutex_t *mutex = perLogFileMutex(index);
    
    CHECK(NULL != mutex,
          "You tried to release the lock for index '"
          << index
          << "' which does not exist!");
          
    pthread_mutex_unlock(mutex);
    
  }//release
  
};//class LogFilePool

template<class L>
const int LogFilePool<L>::arrayMinSize = 10;

struct sLogger {

  ProcessL *activeProcesses;
  
  LogFilePool<ofstream> accessLogs;
  
  LogFilePool<ofstream> testLogs;
  
  LogFilePool<ofstream> oRevisionLogs;
  
  LogFilePool<ifstream> iRevisionLogs;
  
  bool enabled;
  bool replay;
  
  // map from variableID to revision
  MT::Array<VariableHistory*> varHistories;
  
  MT::Array<uint> maxSteps;
  
  // secure structural changes to varHistories
  pthread_mutex_t varHistories_mutex;
  
  
  sLogger() {
  
    pthread_mutex_init(&varHistories_mutex, NULL);
    
    //TODO give proper initialisation. This is only for proper NULL-checks.
    activeProcesses = NULL;
  }
  
  ~sLogger() {
  
    // clean up VariableHistory ressources
    for (uint i=0;
         i < varHistories.N;
         i++) {
      delete varHistories(i);
    }
    
    pthread_mutex_destroy(&varHistories_mutex);
    
  }//~sLogger
  
  //remark: remember to lock any global object before you pass it to others!
  void printVarRevision(const VariableHistory *varHistory,
                        const Variable *i_var) {
                        
    const uint id = i_var->id;
    const bool permissive = true;
    const uint n = varHistory->revisions.N;
    
    ostream &os = testLogs(id, STRING("test/accessLog_" << i_var->name << '_' << id), permissive);
    
    for (uint i = 0; i < n; i++) {
      os << "w,"
      << i
      << ','
      << varHistory->revisions(i).write_pid
      << endl;
      uint m = varHistory->revisions(i).read_pids.N;
      for (uint j = 0; j < m; j++) {
        os << "r,"
        << i
        << ','
        << varHistory->revisions(i).read_pids(j)
        << endl;
        
      }//for r
    }//for w
    
    // free handle
    testLogs.release(id);
  }//printVarRevision
  
  ostream& getAccessLogFile(const Variable *i_var) {
  
    const uint id = i_var->id;
    const bool permissive = true;
    
    ostream *logFile = &accessLogs(id, STRING("log/accessLog_" << i_var->name << '_' << id), permissive);
    
    return *logFile;
  }
  
  /*see getAccessLogFile
   */
  ostream& getORevisionLogFile(const Variable *i_var) {
  
    const uint id = i_var->id;
    const bool permissive = true;
    
    ostream *logFile = &oRevisionLogs(id, STRING("log/revisionLog_" << i_var->name << '_' << id), permissive);
    
    return *logFile;
  }
  
  /*see getAccessLogFile
   */
  istream& getIRevisionLogFile(const Variable *i_var) {
  
    const uint id = i_var->id;
    const bool permissive = true;
    
    // remember: you already OWN the lock!
    istream *logFile = &iRevisionLogs(id, STRING("log/revisionLog_" << i_var->name << '_' << id), permissive);
    
    // ALWAYS return your local copy! Returning a global non-locked value is unpredictable!
    return *logFile;
  }
  
  bool pidBelongsToActiveProccesses(const int pid) {
    CHECK(NULL != activeProcesses, "activeProcesses must be a valid instance (not NULL).");
    
    bool ret = true;
    if (pid != -1) {
      ret = false;
      for (uint i = 0U; i < activeProcesses->N; ++i) {
        if (activeProcesses->elem(i)->id == pid) {
          ret = true;
          break;
        }
      }
    }
    return ret;
  }
  
  void setVariableTransactionsToNextValidState(VariableHistory *i_history, Variable *i_var) {
    if (NULL == activeProcesses)
      return;
    else {
      for (uint rev = i_var->revision + 1; rev < i_history->revisions.N; ++rev) {
        /* find the next revision which can be accessed */
        int writePid = i_history->revisions(rev).write_pid;
        const bool isAccessibleByWrite = pidBelongsToActiveProccesses(writePid);
        if (false != isAccessibleByWrite) {
          /* set to write */
          i_var->revision = MAX(rev - 1, 0);
          i_history->writeRevisionCondVar.setState(i_var->revision);
          break;
        } else {
          /* check the readers */
          bool isAccessibleByRead = false;
          for (uint reader = 0U; reader < i_history->revisions(rev).read_pids.N; ++reader) {
            int readPID = i_history->revisions(rev).read_pids.elem(reader);
            isAccessibleByRead = pidBelongsToActiveProccesses(readPID);
            if (false != isAccessibleByRead) {
              break;
            }
          }
          if (false != isAccessibleByRead) {
            /* set to read */
            i_var->revision = MAX(rev - 1, 0);
            i_history->readRevisionCondVar.setState(i_var->revision);
            break;
          }
        }
      }
    }
  }
  
  VariableHistory* getVariableTransactions(Variable *i_var) {
    uint id = i_var->id;
    
    pthread_mutex_lock(&varHistories_mutex);
    
    while (varHistories.N <= id) {
      varHistories.append(NULL);
    }
    // end of structural changes
    pthread_mutex_unlock(&varHistories_mutex);
    
    /* lock the access internally */
    pthread_mutex_lock(&i_var->replay_mutex);
    
    // maybe someone created the varHistory by now
    if (NULL != varHistories(id)) {
      pthread_mutex_unlock(&i_var->replay_mutex);
      return varHistories(id);
    }
    
    // build fancy access data structure
    VariableHistory *hist = new VariableHistory();
    
    /* set to a state before time ;) */
    hist->readRevisionCondVar.setState(-2);
    hist->writeRevisionCondVar.setState(-2);
    
    /* indicating that the first real (usable) action from the file was read */
    bool firstActionReadIndicator = true;
    
    // hack: revisions start from 1, not zero
    hist->revisions.append();
    
    /* helper to differ which action was performed first */
    bool firstAccessIsRead = false;
    
    // Define the path so it may be used later on for debugs and so forth
    MT::String fileName;
    fileName << "accessLog_" << i_var->name << '_' << id;
    MT::String pathName;
    pathName << "log/" << fileName;
    
    // get access stream
    FILE* accessInputFile = fopen(STRING(pathName), "r");
    if (!accessInputFile)
      HALT("Can't find accessLogFile '" << pathName << "' for var "
           << i_var->name << ' ' << id);
           
    char readOrWrite;
    uint revision;
    int pid;
    uint step_count;
    
    while (EOF != fscanf(accessInputFile, "%c,%u,%d,%d\n", &readOrWrite,
                         &revision, &pid, &step_count)) {
                         
      // we want to extract the last step of each process before termination
      if (pid > -1) {
        uint upid = pid;
        while (maxSteps.N <= upid)
          maxSteps.append(0);
          
        if (maxSteps(upid) < step_count) {
          maxSteps(upid) = step_count;
        }
      }
      
      if ('w' == readOrWrite) {
        // write
        while (hist->revisions.N <= revision) {
          hist->revisions.append();
        }
        
        hist->revisions(revision).write_pid = pid;
        
        if (false != firstActionReadIndicator) {
          firstAccessIsRead = false;
        }
        firstActionReadIndicator = false;
        
      } else if ('r' == readOrWrite) {
        // read
        // Was the 2 parameter append intentionally used?
        hist->revisions(revision).read_pids.append(pid);
        hist->revisions(revision).readsToFinish++;
        
        if (false != firstActionReadIndicator) {
          firstAccessIsRead = true;
        }
        firstActionReadIndicator = false;
      }
      
    }// while
    
    // free file pointer after use
    fclose(accessInputFile);
    
    /* setup the initial replay condition */
    if (false == firstAccessIsRead) {
      /* setup condition variable for the first write action */
      hist->writeRevisionCondVar.setState(0);
      hist->readRevisionCondVar.setState(0);
    } else {
      /* setup condition variable for the first read actions */
      hist->writeRevisionCondVar.setState(-1);
      hist->readRevisionCondVar.setState(0);
    }
    
    // print varRevision to debug
    // printVarRevision(hist, i_var);
    pthread_mutex_lock(&varHistories_mutex);
    varHistories(id) = hist;
    pthread_mutex_unlock(&varHistories_mutex);
    
    pthread_mutex_unlock(&i_var->replay_mutex);
    return hist;
  }
};

Logger::Logger() {
  s = new sLogger;
  //JUMP toggle replay
  s->replay = false;
  s->enabled = true;
}

Logger::~Logger() {
  delete s;
}

void Logger::logReadAccess(const Variable *i_var, const Process *i_p) {

  if (!s->replay && s->enabled) {
  
    const uint id = i_var->id;
    ostream &os = s->getAccessLogFile(i_var);
    
    // format: r/w, revision, pid, step_count
    os  << "r,"
    << i_var->revision
    << ','
    << (i_p ? (int) i_p->id : -1)
    << ','
    << (i_p ? (int) i_p->step_count : 0)
    << endl;
    
    s->accessLogs.release(id);
  }//if replay
  
}//logReadAccess

void Logger::logWriteAccess(const Variable *i_var, const Process *i_p) {

  if (!s->replay && s->enabled) {
  
    const uint id = i_var->id;
    ostream &os = s->getAccessLogFile(i_var);
    
    // format: r/w, revision, pid, step_count
    os  << "w,"
    << i_var->revision
    << ','
    << (i_p ? (int) i_p->id : -1)
    << ','
    << (i_p ? (int) i_p->step_count : 0)
    << endl;
    
    s->accessLogs.release(id);
    
  }//if replay
  
}//logWriteAccess

void Logger::logRevision(const Variable *i_var) {

  if (!s->replay && i_var->logValues && s->enabled) {
  
    const uint id = i_var->id;
    ostream &os = s->getORevisionLogFile(i_var);
    
    MT::String string;
    i_var->serializeToString(string);
    os  << string
    << endl;
    
    s->oRevisionLogs.release(id);
  }
}//logRevision

void Logger::setReplay(const bool replay) {
  // No need to lock this: if set multiple times the value wouldn't change. Correct?
  s->replay = replay;
}

bool Logger::getReplay() const {
  return s->replay;
}//getReplay

void Logger::queryReadAccess(Variable *i_var, const Process *i_p) {

  if (s->replay && s->enabled) {
  
    VariableHistory *history = s->getVariableTransactions(i_var);
    
    pthread_mutex_lock(&history->revisionAccessMutex);
    
    pthread_mutex_lock(&i_var->replay_mutex);
    const uint curRevID = i_var->revision;
    pthread_mutex_unlock(&i_var->replay_mutex);
    
    const int ownPid = ((NULL == i_p)?-1:i_p->id);
    bool findSuccess = false;
    
    s->setVariableTransactionsToNextValidState(history, i_var);
    
    /* find the revision belonging to this read access */
    for (uint revision = curRevID; revision < history->revisions.N; ++revision) {
    
      for (uint j = 0U; j < history->revisions(revision).read_pids.N; ++j) {
      
        if (ownPid == history->revisions(revision).read_pids(j)) {
        
          findSuccess = true;
          /* mark that this process wont access the read anymore */
          history->revisions(revision).read_pids.remove(j);
          
//          pthread_mutex_lock(&g_coutMutex);
//          cout << "query read Access for Var: " << i_var->id << " rev: "
//               << revision << "currentRev: " << i_var->revision
//               << " Process: " << ownPid << endl;
//          pthread_mutex_unlock(&g_coutMutex);

          pthread_mutex_unlock(&history->revisionAccessMutex);
          
          /* this is the next read belonging to me! wait for it to be ready! */
          history->readRevisionCondVar.waitForStateEq(revision);
          /* when we've reached this point we can finally access the data from the variable */
          /* AND it is just like as if we have done so during the actual log time */
          
          break;
        }//if
      }//for
      
      if (findSuccess) break;
    }//for
    
    if (!findSuccess) {
      pthread_mutex_unlock(&history->revisionAccessMutex);
      HALT("Process " << ownPid << " tries to gain unscheduled read access!");
    }
    
    if (i_p && !(i_p->step_count < s->maxSteps(ownPid) - 1)) {
      i_p->s->threadCondition.setState(tsCLOSE);
    }
  }//if replay
}//queryReadAccess

void Logger::queryWriteAccess(Variable *i_var, const Process *i_p) {

  if (s->replay && s->enabled) {
  
    VariableHistory *history = s->getVariableTransactions(i_var);
    
    pthread_mutex_lock(&history->revisionAccessMutex);
    const int ownPid = (NULL != i_p) ? i_p->id : -1;
    bool findSuccess = false;
    
    s->setVariableTransactionsToNextValidState(history, i_var);
    
    pthread_mutex_lock(&i_var->replay_mutex);
    uint revisionId = i_var->revision + 1;
    pthread_mutex_unlock(&i_var->replay_mutex);
    
    /* we want access to the next revision so we have to wait until every read is done. */
    for (;
         revisionId < history->revisions.N;
         ++revisionId) {
         
      /* find the revision belonging to us */
      const int actWritePID = history->revisions(revisionId).write_pid;
      
      if (ownPid == actWritePID) {
//        pthread_mutex_lock(&g_coutMutex);
//        cout  << "query write Access for Var: " << i_var->id << " rev: "
//              << revisionId << " currentRev: " << i_var->revision
//              << " Process: " << ownPid << endl;
//        pthread_mutex_unlock(&g_coutMutex);

        findSuccess = true;
        
        pthread_mutex_unlock(&history->revisionAccessMutex);
        
        /* this is the next read belonging to me! wait for it to be ready! */
        history->writeRevisionCondVar.waitForStateEq(revisionId - 1);
        
        /* when we've reached this point we can finally access the data from the variable */
        /* AND it is just like as if we have done so during the actual log time */
        
        break;
        
      }//if pid found
      
    }//for
    
//    pthread_mutex_lock(&g_coutMutex);
//    cout  << "write Access acquired for Var: " << i_var->id << " rev: "
//          << i_var->revision << " Process: " << ownPid << endl;
//    pthread_mutex_unlock(&g_coutMutex);

    if (!findSuccess) {
      pthread_mutex_unlock(&history->revisionAccessMutex);
      HALT("Process " << ownPid << " tries to gain unscheduled write access!");
    }
    
    if (i_p && !(i_p->step_count < s->maxSteps(ownPid) - 1)) {
//      cout << "Process  " << ownPid << " is done.";
      i_p->s->threadCondition.setState(tsCLOSE);
    }
    
  }//if replay
  
}//queryWriteAcess

void Logger::freeWriteAccess(Variable *i_var) {

  if (s->replay && s->enabled) {
  
    VariableHistory *history = s->getVariableTransactions(i_var);
    
    pthread_mutex_lock(&history->revisionAccessMutex);
    
    /* let all the readers go */
    if (NULL == s->activeProcesses) {
      history->readRevisionCondVar.setState(i_var->revision);
      /* if there is no reader then let the next writer go */
      if (0U == history->revisions(i_var->revision).read_pids.N) {
        history->writeRevisionCondVar.setState(i_var->revision);
      }
    } else {
      s->setVariableTransactionsToNextValidState(history, i_var);
    }
    pthread_mutex_unlock(&history->revisionAccessMutex);
  }//if replay
}//freeWriteAccess

void Logger::freeReadAccess(Variable *i_var) {

  if (s->replay && s->enabled) {
    VariableHistory *history = s->getVariableTransactions(i_var);
    
    if (i_var->revision >= history->revisions.N)
      return;
      
    pthread_mutex_lock(&history->revisionAccessMutex);
    uint revision = i_var->revision;
    
    if (NULL == s->activeProcesses) {
    
      /* tell the world that one reader is done */
      pthread_mutex_lock(&history->revisions(revision).readsToFinishMutex);
      --(history->revisions(revision).readsToFinish);
      
      /* if we are the last read access then we tell the next writer to go */
      if (0U == history->revisions(revision).readsToFinish) {
        history->writeRevisionCondVar.setState(revision);
      }//if no more reads
      
      pthread_mutex_unlock(&history->revisions(revision).readsToFinishMutex);
    } else {
      s->setVariableTransactionsToNextValidState(history, i_var);
    }
    
    pthread_mutex_unlock(&history->revisionAccessMutex);
  }//if replay
}//freeReadAccess

void Logger::setValueIfDbDriven(Variable *i_var) {

  if (s->replay && i_var->dbDrivenReplay && s->enabled) {
    const int id = i_var->id;
    
    // get string containing field info from log
    MT::String string;
    istream &is = s->getIRevisionLogFile(i_var);
    string.read(is, "", "\n");
    is.sync();
    s->iRevisionLogs.release(id);
    
    // deserialize string
    i_var->deSerializeFromString(string);
    
  }//if replay
}//setValueIfDbDriven

// TODO NEVER CALLED!
//void Logger::setActiveProcesses(ProcessL *i_list) {
//  s->activeProcesses = i_list;
//}

#else //BIROS_LOGGER

Logger::Logger(){};
Logger::~Logger(){};
//methods called by the user
void Logger::logRevision(const Variable *i_var){};
void Logger::setReplay(const bool replay){};
bool Logger::getReplay() const {};

//methods called during write/read access from WITHIN biros
void Logger::logReadAccess(const Variable *i_var, const Process *i_p){};
void Logger::logWriteAccess(const Variable *i_var, const Process *i_p){};
void Logger::queryReadAccess(Variable *i_var, const Process *i_p){};
void Logger::queryWriteAccess(Variable *i_var, const Process *i_p){};
void Logger::freeWriteAccess(Variable *i_var){};
void Logger::freeReadAccess(Variable *i_var){};
void Logger::setValueIfDbDriven(Variable *i_var){};

Logger logService; // TODO no global stuff

#endif