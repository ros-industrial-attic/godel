#include "godel_simple_gui/parameter_window_base.h"
#include <ros/console.h>

void godel_simple_gui::ParameterWindowBase::show()
{
  update_display_fields();
  QWidget::show();
}

void godel_simple_gui::ParameterWindowBase::accept_changes_handler()
{
  this->update_internal_fields();
  Q_EMIT parameters_changed();
  hide();
}

void godel_simple_gui::ParameterWindowBase::cancel_changes_handler() { hide(); }

void godel_simple_gui::ParameterWindowBase::save_changes_handler()
{
  accept_changes_handler();
  Q_EMIT parameters_save_requested();
}
