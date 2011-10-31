StateVariable::StateVariable() {
  id=globalVars.N;
  globalVars.append(this);
  dim=0;
}

void StateVariable::write(std::ostream& os) const {
  os <<name;
  if (valueNames.N) {
    os <<' ';
    valueNames.write(os," ","\n ","[]");
  } else if (dim!=2) os <<" <" <<dim <<'>';
  os <<std::endl;
}

void StateVariable::read(istream& is) {
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
  StateVariable *s;
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
    s = listFindByName(globalVars,varName);
    if (!s) {
      int r;
      uint j;
      for_list(j,s,globalVars) {
        r=s->valueNames.findValue(varName);
        if (r>=0) {
          values.append(r);
          break;
        }
      }
    } else {
      values.append(1);
    }
    if (!s) HALT("don't know variable or value '"<<varName <<"'");
    vars.append(s);
  }
  weight = 1.;
}
