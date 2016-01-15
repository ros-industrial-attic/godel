#include <godel_plugins/widgets/surface_detection_configuration.h>

#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*0.017453293)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x)*57.29578)
#endif

godel_plugins::SurfaceDetectionConfigWidget::SurfaceDetectionConfigWidget(
    const godel_msgs::SurfaceDetectionParameters& params)
    : params_(params)
{
  ui_.setupUi(this);

  connect(ui_.PushButtonAccept, SIGNAL(clicked()), this, SLOT(accept_changes_handler()));
  connect(ui_.PushButtonCancel, SIGNAL(clicked()), this, SLOT(cancel_changes_handler()));
  connect(ui_.PushButtonSave, SIGNAL(clicked()), this, SLOT(save_changes_handler()));
}

void godel_plugins::SurfaceDetectionConfigWidget::update_gui_fields()
{
  ui_.LineEditFrameId->setText(QString::fromStdString(params_.frame_id));
  ui_.LineEditKSearch->setText(QString::number(params_.k_search));
  ui_.LineEditMarkerAlpha->setText(QString::number(params_.marker_alpha));

  ui_.LineEditStOutMean->setText(QString::number(params_.meanK));
  ui_.LineEditStOutThreshold->setText(QString::number(params_.stdv_threshold));

  ui_.LineEditRgMinClusterSize->setText(QString::number(params_.rg_min_cluster_size));
  ui_.LineEditRgMaxClusterSize->setText(QString::number(params_.rg_max_cluster_size));
  ui_.LineEditRgNeighbors->setText(QString::number(params_.rg_neightbors));
  ui_.LineEditRgSmoothnessThreshold->setText(
      QString::number(RAD2DEG(params_.rg_smoothness_threshold)));
  ui_.LineEditRgCurvatureThreshold->setText(QString::number(params_.rg_curvature_threshold));

  ui_.LineEditVoxelLeaf->setText(QString::number(params_.voxel_leafsize));
  ui_.LineEditTabletopSegmentationDist->setText(
      QString::number(params_.tabletop_seg_distance_threshold));
  ui_.CheckBoxUseTabletopSegmentation->setChecked(static_cast<bool>(params_.use_tabletop_seg));
  ui_.CheckBoxIgnoreLargestCluster->setChecked(static_cast<bool>(params_.ignore_largest_cluster));

  ui_.LineEditMlsPointDensity->setText(QString::number(params_.mls_point_density));
  ui_.LineEditMlsUpsamplingRadius->setText(QString::number(params_.mls_upsampling_radius));
  ui_.LineEditMlsSearchRadius->setText(QString::number(params_.mls_search_radius));

  ui_.LineEditTrSearchRadius->setText(QString::number(params_.tr_search_radius));
  ui_.LineEditTrMu->setText(QString::number(params_.tr_mu));
  ui_.LineEditTrNearestNeighbors->setText(QString::number(params_.tr_max_nearest_neighbors));
  ui_.LineEditTrMaxSurfaceAngle->setText(QString::number(RAD2DEG(params_.tr_max_surface_angle)));
  ui_.LineEditTrMinAngle->setText(QString::number(RAD2DEG(params_.tr_min_angle)));
  ui_.LineEditTrMaxAngle->setText(QString::number(RAD2DEG(params_.tr_max_angle)));
  ui_.CheckBoxTrNormalConsistency->setChecked(static_cast<bool>(params_.tr_normal_consistency));

  ui_.CheckBoxPaEnabled->setChecked(static_cast<bool>(params_.pa_enabled));
  ui_.LineEditPaSegMaxIterations->setText(QString::number(params_.pa_seg_max_iterations));
  ui_.LineEditPaSegDistThreshold->setText(QString::number(params_.pa_seg_dist_threshold));
  ui_.LineEditPaSACPlaneDistance->setText(QString::number(params_.pa_sac_plane_distance));
  ui_.LineEditPaKdtreeRadius->setText(QString::number(params_.pa_kdtree_radius));
}

void godel_plugins::SurfaceDetectionConfigWidget::update_internal_values()
{
  params_.frame_id = ui_.LineEditFrameId->text().toStdString();
  params_.k_search = ui_.LineEditKSearch->text().toDouble();
  params_.marker_alpha = ui_.LineEditMarkerAlpha->text().toDouble();

  params_.meanK = ui_.LineEditStOutMean->text().toDouble();
  params_.stdv_threshold = ui_.LineEditStOutThreshold->text().toDouble();

  params_.rg_min_cluster_size = ui_.LineEditRgMinClusterSize->text().toDouble();
  params_.rg_max_cluster_size = ui_.LineEditRgMaxClusterSize->text().toDouble();
  params_.rg_neightbors = ui_.LineEditRgNeighbors->text().toDouble();
  params_.rg_smoothness_threshold = DEG2RAD(ui_.LineEditRgSmoothnessThreshold->text().toDouble());
  params_.rg_curvature_threshold = ui_.LineEditRgCurvatureThreshold->text().toDouble();

  params_.voxel_leafsize = ui_.LineEditVoxelLeaf->text().toDouble();
  params_.tabletop_seg_distance_threshold = ui_.LineEditTabletopSegmentationDist->text().toDouble();
  params_.use_tabletop_seg = ui_.CheckBoxUseTabletopSegmentation->isChecked();
  params_.ignore_largest_cluster = ui_.CheckBoxIgnoreLargestCluster->isChecked();

  params_.mls_point_density = ui_.LineEditMlsPointDensity->text().toDouble();
  params_.mls_upsampling_radius = ui_.LineEditMlsUpsamplingRadius->text().toDouble();
  params_.mls_search_radius = ui_.LineEditMlsSearchRadius->text().toDouble();

  params_.tr_search_radius = ui_.LineEditTrSearchRadius->text().toDouble();
  params_.tr_mu = ui_.LineEditTrMu->text().toDouble();
  params_.tr_max_nearest_neighbors = ui_.LineEditTrNearestNeighbors->text().toDouble();
  params_.tr_max_surface_angle = DEG2RAD(ui_.LineEditTrMaxSurfaceAngle->text().toDouble());
  params_.tr_min_angle = DEG2RAD(ui_.LineEditTrMinAngle->text().toDouble());
  params_.tr_max_angle = DEG2RAD(ui_.LineEditTrMaxAngle->text().toDouble());
  params_.tr_normal_consistency = ui_.CheckBoxTrNormalConsistency->isChecked();

  params_.pa_enabled = ui_.CheckBoxPaEnabled->isChecked();
  params_.pa_seg_max_iterations = ui_.LineEditPaSegMaxIterations->text().toInt();
  params_.pa_seg_dist_threshold = ui_.LineEditPaSegDistThreshold->text().toDouble();
  params_.pa_sac_plane_distance = ui_.LineEditPaSACPlaneDistance->text().toDouble();
  params_.pa_kdtree_radius = ui_.LineEditPaKdtreeRadius->text().toDouble();
}
