#ifndef MT_MSVC
#  include <sys/ipc.h>
#  include <sys/shm.h>
#endif

#ifdef MT_MSVC
struct SHM {
  HANDLE systemId;
  void *p;
  bool created, opened;
  SHM(){ created=opened=false; p=NULL; systemId=NULL; }
};

void *openSharedMemory(SHM& shm, const char *name, uint size){
  CHECK(!created && !opened, "shm object already in use");
  
  systemId = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, name);
  if(systemId == NULL){
    systemId = CreateFileMapping(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, size, name);
    std::cout <<"** creating new shared memory `" <<name <<"' with systemId " <<systemId <<std::endl;
    if(systemId==NULL)
      HALT("can't create shared memory `" <<name <<"'. CreateFileMapping error code " <<GetLastError());
    created=true;
  }else{
    std::cout <<"** opening existing shared memory `" <<name <<"' with systemId " <<systemId <<std::endl;
  }
  
  p = MapViewOfFile(systemId, FILE_MAP_ALL_ACCESS, 0, 0, size);
  if(p==NULL)
    HALT("Could not map view of file. MapViewOfFile error code " <<GetLastError());
    
  opened=true;
  return p;
}

void closeSharedMemory(SHM& shm){
  CHECK(opened, "can't close what's not open");
  UnmapViewOfFile(p);
  CloseHandle(systemId);
  systemId=NULL;
  p=NULL;
  created=opened=false;
}

void destroySharedMemory(SHM& shm){
  std::cout <<"** destroying shared memory with systemId " <<systemId <<" -- not supported under MSVC" <<std::endl;
}

#else
#ifdef MT_PTHREAD
void SHM::open(const char *name, uint size, bool directDestroy){
  key_t key=ftok(name, 0);
  systemId=shmget(key, size, 0666);
  if(systemId==-1){
    systemId=shmget(key, size, IPC_CREAT|0666);
    std::cout <<"** creating new shared memory `" <<name <<"' with systemId " <<systemId <<std::endl;
    if(systemId==-1)
      HALT("can't create shared memory `" <<name);
    created=true;
  }else{
    std::cout <<"** opening existing shared memory `" <<name <<"' with systemId " <<systemId <<std::endl;
  }
  opened=true;

  if(directDestroy){ destroy(); return; }

  p=(byte*)shmat(systemId, NULL, 0);
  if(p==(byte*) -1)
    HALT("can't access memory of shared memory `" <<name);

  info=(SHMinfoBlock*)p;
  uint i;
  if(created){
    if(size<=sizeof(SHMinfoBlock)) HALT("need to allocate larger SHM to hold info block!");
    strcpy((char*)info->pagename, name);
    info->size=size;
    info->used=sizeof(SHMinfoBlock);
    info->guests=1;
    guestId=0;
    for(i=0; i<maxShmGuests; i++) info->guestPids[i]=-1;
    info->guestPids[guestId]=getpid();
    for(i=0; i<shmMaxBlocks; i++) info->blockNames[i][0]=0;
    for(i=0; i<shmMaxBlocks; i++) info->blockOffsets[i]=0;
    //create mutexes for all (potential) blockes:
    pthread_rwlockattr_t   att;
    int r;
    r=pthread_rwlockattr_init(&att); CHECK(!r, "");
    r=pthread_rwlockattr_setpshared(&att, PTHREAD_PROCESS_SHARED); CHECK(!r, "");
    for(i=0; i<shmMaxBlocks; i++){
      r=pthread_rwlock_init(&info->blockLocks[i], &att);
      CHECK(!r, "");
    }
  }else{
    //CHECK(!strcmp(name, info->pagename), "requested shm has different name");
    //CHECK(size==info->size, "requested shm has different size");
    info->guests++;
    for(i=0; i<maxShmGuests; i++) if(info->guestPids[i]==-1) break;
    if(i==maxShmGuests) HALT("exceeded max number of clients " <<maxShmGuests);
    guestId=i;
    info->guestPids[guestId]=getpid();
  }
  opened=true;

  report();
}

void SHM::close(){
  CHECK(opened, "can't close what's not open");
  info->guests--;
  info->guestPids[guestId]=-1;
  shmdt(p);
  created=opened=false; p=NULL; systemId=NULL; info=NULL; guestId=-1;
}

void SHM::destroy(){
  std::cout <<"** destroying shared memory with systemId " <<systemId <<std::endl;
  shmid_ds buf;
  if(shmctl(systemId, IPC_RMID, &buf)<0){
    HALT("failed destroying the shm");
  }
  created=opened=false; p=NULL; systemId=NULL; info=NULL; guestId=-1;
}

//documentation: http://cs.pub.ro/~apc/2003/resources/pthreads/uguide/document.htm
void SHM::readLock(uint i){
  int r=pthread_rwlock_rdlock(&info->blockLocks[i]);
  if(r) HALT("");
}

void SHM::writeLock(uint i){
  int r=pthread_rwlock_wrlock(&info->blockLocks[i]);
  if(r) HALT("");
}

void SHM::unlock(uint i){
  int r=pthread_rwlock_unlock(&info->blockLocks[i]);
  if(r) HALT("");
}

void SHM::report(){
  std::cout <<"** SHM report:";
  if(!opened){ std::cout <<" -- not open" <<std::endl; return; }
  uint i;
  std::cout
    <<"\n  pagename=" <<info->pagename
    <<"\n  size=" <<info->size
    <<"\n  used=" <<info->used
    <<"\n  guests=" <<info->guests
    <<"\n  guestPids=";
  for(i=0; i<maxShmGuests; i++) std::cout <<' ' <<info->guestPids[i];
  std::cout
    <<"\n  local guestId=" <<guestId
    <<"\n  local systemId=" <<systemId
    <<"\n  local p=" <<(void*)p
    <<"\n  locally created=" <<created
    <<"\n  allocated blocks:";
  for(i=0; i<shmMaxBlocks; i++) if(info->blockOffsets[i]) std::cout <<"\n    " <<i <<": name=" <<info->blockNames[i] <<" offset=" <<info->blockOffsets[i];
  std::cout <<std::endl;
}
#endif //PTHREAD
#endif
