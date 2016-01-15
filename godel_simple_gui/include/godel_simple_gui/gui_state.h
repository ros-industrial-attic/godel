#ifndef SIMPLE_GUI_STATE_H
#define SIMPLE_GUI_STATE_H

#include <QObject>

namespace godel_simple_gui
{

// Forward declare Main Widget
class BlendingWidget;

class GuiState : public QObject
{
  Q_OBJECT
public:
  virtual ~GuiState() {}

  // Entry and exit classes
  virtual void onStart(BlendingWidget& gui) = 0;
  virtual void onExit(BlendingWidget& gui) = 0;

  // Handlers for the fixed buttons
  virtual void onNext(BlendingWidget& gui) = 0;
  virtual void onBack(BlendingWidget& gui) = 0;
  virtual void onReset(BlendingWidget& gui) = 0;

Q_SIGNALS:
  void newStateAvailable(GuiState*);
};
}

#endif
