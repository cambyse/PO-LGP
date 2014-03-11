template<typename T>
void G4Data::appendMeta(const char *name, const T &data) {
  cout << " * Appending meta: " << name << endl;
  kvg.append("meta", name, new T(data));
}
