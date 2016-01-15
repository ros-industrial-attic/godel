#ifndef PARAMETER_WINDOW_BASE_H
#define PARAMETER_WINDOW_BASE_H

#include <QWidget>

namespace godel_simple_gui
{

/**
 * @brief Abstract base class for parameter windows in the blending widget
 */
class ParameterWindowBase : public QWidget
{
  Q_OBJECT

public:
  /**
   * @brief Updates GUI parameters and shows the main window
   */
  virtual void show();

  /**
   * @brief Reads the internal data structure to update the fields of the GUI
   */
  virtual void update_display_fields() = 0;

  /**
   * @brief Reads the fields of the GUI to update the internal data structure
   */
  virtual void update_internal_fields() = 0;

Q_SIGNALS:
  /**
   * @brief Indicates an edit of the GUI interface by the user
   */
  void parameters_changed();
  void parameters_save_requested();

protected Q_SLOTS:

  virtual void accept_changes_handler();
  virtual void cancel_changes_handler();
  virtual void save_changes_handler();
};
}

#endif // PARAMETER_WINDOW_BASE_H
