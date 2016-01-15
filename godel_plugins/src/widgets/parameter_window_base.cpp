#include "godel_plugins/widgets/parameter_window_base.h"

void godel_plugins::ParameterWindowBase::show()
{
  update_gui_fields();
  QMainWindow::show();
}

void godel_plugins::ParameterWindowBase::accept_changes_handler()
{
  this->update_internal_values();
  Q_EMIT parameters_changed();
  hide();
}

void godel_plugins::ParameterWindowBase::cancel_changes_handler() { hide(); }

void godel_plugins::ParameterWindowBase::save_changes_handler()
{
  accept_changes_handler();
  Q_EMIT parameters_save_requested();
}
