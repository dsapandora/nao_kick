/**
 * @author Emilie Wirbel
 *
 * This file was generated by Aldebaran Robotics ModuleGenerator
 */

#ifndef BUMPER_BUMPER_H
#define BUMPER_BUMPER_H

#include <boost/shared_ptr.hpp>
#include <alcommon/almodule.h>
#include <string>

#include <alproxies/almemoryproxy.h>
#include <alproxies/altexttospeechproxy.h>
#include <althread/almutex.h>

namespace AL
{
  class ALBroker;
}

class Bumper : public AL::ALModule
{
  public:

    Bumper(boost::shared_ptr<AL::ALBroker> broker, const std::string& name);

    virtual ~Bumper();

    /** Overloading ALModule::init().
    * This is called right after the module has been loaded
    */
    virtual void init();

    /**
    * This method will be called every time the event RightBumperPressed is raised.
    */
    void onRightBumperPressed();
    void onLeftBumperPressed();

  private:
    AL::ALMemoryProxy fMemoryProxy;
    AL::ALMemoryProxy fMemoryProxy2;

    AL::ALTextToSpeechProxy fTtsProxy;
    AL::ALTextToSpeechProxy fTtsProxy2;

    boost::shared_ptr<AL::ALMutex> fCallbackMutex;

    float fState;
    float fState2;
};
#endif  // BUMPER_BUMPER_H

