template<class T> Link<T>::Link(Process *_p){
  p = _p;
  last_revision = 0;
}

template<class T> bool Link<T>::needsUpdate(){
  MT_MSG("does this need a lock???");
  return last_revision != var->revision;
}

template<class T> void Link<T>::writeAccess(){
  var->writeAccess(p);
}

template<class T> void Link<T>::readAccess(){
  var->readAccess(p);
}

template<class T> void Link<T>::deAccess(){
  last_revision = var->revision;
  var->deAccess(p);
}

