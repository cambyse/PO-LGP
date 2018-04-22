#pragma once

namespace matp
{
class IncoherentDefinition: public std::exception
{
  virtual const char* what() const throw()
  {
    return "Incoherent definition";
  }
};

// constants
const std::string agentPrefix_  = "__AGENT_";
const std::string agentSuffix_  = "__";

} // namespace matp
