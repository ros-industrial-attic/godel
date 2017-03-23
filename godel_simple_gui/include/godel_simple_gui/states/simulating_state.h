#ifndef SIMULATING_STATE_H
#define SIMULATING_STATE_H

#include "godel_simple_gui/gui_state.h"
#include <ros/ros.h>

namespace godel_simple_gui
{

class SimulatingState : public GuiState
{
  Q_OBJECT
public:
  // Constructor
  SimulatingState(const std::vector<std::string>& plans);

  // Entry and exit classes
  virtual void onStart(BlendingWidget& gui);
  virtual void onExit(BlendingWidget& gui);

  // Handlers for the fixed buttons
  virtual void onNext(BlendingWidget& gui);
  virtual void onBack(BlendingWidget& gui);
  virtual void onReset(BlendingWidget& gui);

protected:
  void simulateAll(BlendingWidget& gui);
  void simulateOne(const std::string& plan, BlendingWidget& gui);

private:
  std::vector<std::string> plan_names_;
  ros::ServiceClient sim_client_;
};
}

#endif // SIMULATING_STATE_H
