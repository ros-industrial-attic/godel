#ifndef SIMPLE_GUI_BLENDING_PANEL_H
#define SIMPLE_GUI_BLENDING_PANEL_H

#include "rviz/panel.h"

namespace godel_simple_gui
{

// Forward declare blend widget
class BlendingWidget;

class BlendingPanel : public rviz::Panel
{
  Q_OBJECT
public:
  BlendingPanel(QWidget* parent = 0);

  virtual ~BlendingPanel();

  virtual void onInitialize();

protected:
  BlendingWidget* widget_;
};
}

#endif
