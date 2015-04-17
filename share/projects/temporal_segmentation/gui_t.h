// FGP {{{
template<class T>
void FGP::set(const char *key, const T &value) {
  params.set<T>(key, value);
}
// }}}
