//===========================================================================
//
// a shared memory
//

namespace MT {

#define maxShmGuests 20
#define shmMaxBlocks 20
struct SHMinfoBlock {
  const char pagename[20];
  uint size, used;
  uint guests;
  int guestPids[maxShmGuests];
  char blockNames[shmMaxBlocks][20];
  uint blockOffsets[shmMaxBlocks];
  pthread_rwlock_t blockLocks[shmMaxBlocks];
};

struct RWLock {
  //documentation: http://cs.pub.ro/~apc/2003/resources/pthreads/uguide/document.htm
  pthread_rwlock_t lock;
  void init(bool process_shared=true){
    pthread_rwlockattr_t   att;
    int r;
    r=pthread_rwlockattr_init(&att); if(r) HALT("");
    if(process_shared){
      r=pthread_rwlockattr_setpshared(&att, PTHREAD_PROCESS_SHARED); if(r) HALT("");
    }
    r=pthread_rwlock_init(&lock, &att); if(r) HALT("");
  }
  void read(){
    int r=pthread_rwlock_rdlock(&lock);
    if(r) HALT("");
  }
  
  void write(){
    int r=pthread_rwlock_wrlock(&lock);
    if(r) HALT("");
  }
  
  void unlock(){
    int r=pthread_rwlock_unlock(&lock);
    if(r) HALT("");
  }
};

struct SHM {
  int systemId;
  uint guestId;
  byte *p;
  bool created, opened;
  SHMinfoBlock *info;
  SHM(){ created=opened=false; p=NULL; systemId=NULL; info=NULL; guestId=(uint)-1; }
  void open(const char *pagename, uint size, bool directDestroy=false);
  void close();
  void destroy();
  void readLock(uint i);
  void writeLock(uint i);
  void unlock(uint i);
  void report();
  template<class T> T* newBlock(const char* objName, uint *i=NULL);
};

template<class T> struct Shared {
  uint blockId;
  T *p;
  SHM *shm;
  Shared(SHM& _shm, const char* name){ shm=&_shm; p=shm->newBlock<T>(name, &blockId); }
  void readLock(){ shm->readLock(blockId); }
  void writeLock(){ shm->writeLock(blockId); }
  void unlock()   { shm->unlock(blockId); }
  void set(const T& x){ writeLock(); *p=x; unlock(); }
  void get(T& x)      { readLock();  x=*p; unlock(); }
  T& operator()(){ return *p; }
};
}

template<class T> T* MT::SHM::newBlock(const char* objName, uint *_i){
  if(!opened) open("shmPageName", 1000);
  uint i;
  for(i=0; i<shmMaxBlocks; i++) if(!strcmp(objName, info->blockNames[i])) break;
  if(i==shmMaxBlocks){ //allocate new
    for(i=0; i<shmMaxBlocks; i++) if(!info->blockOffsets[i]) break;
    if(i==shmMaxBlocks) HALT("exceeded number of shm blocks");
    strcpy(info->blockNames[i], objName);
    info->blockOffsets[i] = info->used;
    info->used += sizeof(T);
    if(info->used>info->size) HALT("not enough space in shared memory");
  }
  T *x=(T*)(p+info->blockOffsets[i]);
  if(_i) *_i=i;
  return x;
}
