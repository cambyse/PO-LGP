#include "gui.h"

// FGP {{{
FGP::FGP() {
  setDefaultParams();
}

void FGP::setDefaultParams() {
  defParams.clear();
  defParams.set("lines", false);
  defParams.set("points", false);
  defParams.set("domain", false);
  defParams.set("dataid", false);
  defParams.set("3d", false);
  defParams.set("autolegend", false);
  defParams.set("title", (const char *)NULL);
  defParams.set("hardcopy", (const char *)NULL);
  defParams.set("stream", -1.);
  defParams.set("ymin_b", false);
  defParams.set("ymax_b", false);
  defParams.set("ymin", 0.);
  defParams.set("ymax", 0.);
  defParams.set("dump", false);
  reset();
}

void FGP::reset() {
  // TODO  only reset one parameter? hard.. because of the type which you just don't know..
  params = defParams;
}

void FGP::open() {
  std::stringstream cmd;
  cmd << "feedgnuplot";
  cmd << (*params.get<bool>("lines")? " --lines": " --nolines");
  cmd << (*params.get<bool>("points")? " --points": " --nopoints");
  cmd << (*params.get<bool>("domain")? " --domain": " --nodomain");
  cmd << (*params.get<bool>("dataid")? " --dataid": " --nodataid");
  cmd << (*params.get<bool>("3d")? " --3d": " --no3d");
  if(*params.get<bool>("autolegend"))
    cmd << " --autolegend";
  if(*params.get<const char *>("title"))
    cmd << " --title '" << *params.get<const char *>("title") << "'";
  if(*params.get<const char *>("hardcopy"))
    cmd << " --hardcopy " << *params.get<const char *>("hardcopy");
  if(*params.get<double>("stream") >= 0)
    cmd << " --stream " << *params.get<double>("stream");
  if(*params.get<bool>("ymin_b"))
    cmd << " --ymin " << *params.get<double>("ymin");
  if(*params.get<bool>("ymax_b"))
    cmd << " --ymax " << *params.get<double>("ymax");
  if(*params.get<bool>("dump"))
    cmd << " --dump";
  cmd << " --exit";
  /* cmd << " &2>/dev/null"; */

  f = popen(cmd.str().c_str(), "w");
  CHECK(f, "popen() failed to connect to feedgnuplot.");
}

void FGP::plot() {
  operator()() << "replot";
}

void FGP::close() {
  if(f) {
    pclose(f);
    f = NULL;
  }
}

Collector FGP::operator()() {
  return Collector([this](const std::string &s){
      fprintf(f, "%s\n", s.c_str());
      fflush(f);
      });
  /* return StreamCollector(f); */
}
// }}}

