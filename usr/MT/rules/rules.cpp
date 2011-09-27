Variable::Variable() {
  id=globalVars.N;
  globalVars.append(this);
  dim=0;
}

void Variable::write(std::ostream& os) const {
  os <<name;
  if (valueNames.N) {
    os <<' ';
    valueNames.write(os," ","\n ","[]");
  } else if (dim!=2) os <<" <" <<dim <<'>';
  os <<std::endl;
}

void Variable::read(istream& is) {
  MT::String::readStopSymbols = " []<>\n\r";
  MT::String::readEatStopSymbol = 0;
  is >>name;
  char c=MT::peerNextChar(is);
  if (c=='[') {
    is >>valueNames;
    dim=valueNames.N;
    return;
  }
  if (c=='<') {
    is >>"<" >>dim >>">";
    return;
  }
  dim = 2;
}

void Rule::write(std::ostream& os) const {
  uint i;
  for (i=0; i<arrow; i++) {
    if (negation(i)) os <<'-';
    os <<vars(i)->name;
    if (values(i)!=1 || vars(i)->valueNames.N) os <<'=' <<vars(i)->valueNames(values(i));
    os <<' ';
  }
  os <<" > ";
  for (; i<vars.N; i++) {
    if (negation(i)) os <<'-';
    os <<vars(i)->name;
    if (values(i)!=1 || vars(i)->valueNames.N) os <<'=' <<vars(i)->valueNames(values(i));
    os <<' ';
  }
  os <<endl;
}

void Rule::read(std::istream& is) {
  MT::String::readStopSymbols = " []<>\n\r";
  MT::String::readEatStopSymbol = 0;
  uint i;
  char c;
  MT::String varName;
  Variable *v;
  for (i=0;; i++) {
    c=MT::peerNextChar(is," \r\t");
    if (c=='\n') break;
    if (c=='-') {
      is.get(c);
      negation.append(true);
    } else negation.append(false);
    if (c=='>') {
      is.get(c);
      arrow=i;
      continue;
    }
    is >>varName;
    v = listFindByName(globalVars,varName);
    if (!v) {
      int r;
      uint j;
      for_list(j,v,globalVars) {
        r=v->valueNames.findValue(varName);
        if (r>=0) {
          values.append(r);
          break;
        }
      }
    } else {
      values.append(1);
    }
    if (!v) HALT("don't know variable or value '"<<varName <<"'");
    vars.append(v);
  }
  weight = 1.;
}
