#ifndef BLEND_TOOL_PARAM_WINDOW_H
#define BLEND_TOOL_PARAM_WINDOW_H

#include <QWidget>

namespace Ui {
class BlendToolParamWindow;
}

class BlendToolParamWindow : public QWidget
{
  Q_OBJECT

public:
  explicit BlendToolParamWindow(QWidget *parent = 0);
  ~BlendToolParamWindow();

private slots:
  void on_save_request();

private:
  Ui::BlendToolParamWindow *ui;
};

#endif // BLEND_TOOL_PARAM_WINDOW_H
