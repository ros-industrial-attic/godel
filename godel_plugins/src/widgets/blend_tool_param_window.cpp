#include "godel_plugins/widgets/blend_tool_param_window.h"
#include "ui_blend_tool_param_window.h"

BlendToolParamWindow::BlendToolParamWindow(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::BlendToolParamWindow)
{
  ui->setupUi(this);

  connect(ui->pushButton_2, SIGNAL(clicked()), this, SLOT(on_save_request()));
}

BlendToolParamWindow::~BlendToolParamWindow()
{
  delete ui;
}

void BlendToolParamWindow::on_save_request()
{
}
