#include "bota_signal_handler/SignalHandler.hpp"

namespace signal_handler
{
/*****************************************************************************/
/* Static Member Initialization                                              */
/*****************************************************************************/

std::map<int, std::list<SignalHandler::Handler> > SignalHandler::handlers_;
std::mutex SignalHandler::mutex_;

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void SignalHandler::bind(int signal_, const Handler& handler)
{
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = handlers_.find(signal_);
  if (it == handlers_.end())
  {
    it = handlers_.emplace(std::make_pair(signal_, std::list<Handler>())).first;
    signal(signal_, &SignalHandler::signaled);
  }

  for (auto& jt : it->second)
  {
    if (jt.target_type().name() == handler.target_type().name())
    {
      return;
    }
  }

  it->second.push_back(handler);
}

void SignalHandler::unbind(int signal_, const Handler& handler)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = handlers_.find(signal_);

  if (it == handlers_.end())
  {
    return;
  }

  for (auto jt = it->second.begin(); jt != it->second.end(); ++jt)
  {
    if (jt->target_type().name() == handler.target_type().name())
    {  // jt->target
      it->second.erase(jt);

      if (it->second.empty())
      {
        handlers_.erase(it);
        signal(signal_, SIG_DFL);
      }

      return;
    }
  }
}

void SignalHandler::signaled(int signal)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = handlers_.find(signal);

  if (it == handlers_.end())
  {
    return;
  }

  for (auto& jt : it->second)
  {
    jt(signal);
  }
}

}  // namespace signal_handler
