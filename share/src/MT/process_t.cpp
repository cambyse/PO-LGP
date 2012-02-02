template<class T> Link::Link(Process *p){
  process = p;
  last_revision = 0;
}

bool template<class T> Link::needsUpdate(){
  MT_MSG("does this need a lock???");
  return last_revision != var->revision;
}

template<class T> Link::writeAccess(){
  var->writeAccess(p);
}

template<class T> Link::readAccess(){
  var->readAccess(p);
}

template<class T> Link::deAccess(){
  last_revision = var->revision;
  var->deAccess(p);
}

