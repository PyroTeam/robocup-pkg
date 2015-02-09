#ifndef rqt_ft_gui__FtGui_H
#define rqt_ft_gui__FtGui_H

#include <rqt_gui_cpp/plugin.h>

#include <ui_ft_gui.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

#include <opencv2/core/core.hpp>

#include <QImage>
#include <QList>
#include <QMutex>
#include <QString>
#include <QSize>
#include <QWidget>

#include <vector>
#include <XmlRpcValue.h>

namespace rqt_ft_gui {

class FtGui
  : public rqt_gui_cpp::Plugin
{

  Q_OBJECT

public:

  FtGui();

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);

  virtual bool eventFilter(QObject* watched, QEvent* event);

  virtual void shutdownPlugin();

  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;

  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

protected slots:

  virtual void updateTopicList();
  virtual void updateTopicList_2();

protected:

  virtual QList<std::pair<QString,QString> > getTopicList(const QSet<QString>& message_types, const QList<QString>& transports);
  virtual QList<QString> getTopicList_2(const QSet<QString>& message_types, const QList<QString>& transports);

  virtual void selectTopic(const QString& topic);
  virtual void selectTopic_2(const QString& topic);

protected slots:

  virtual void onTopicChanged(int index);
  virtual void onTopicChanged_2(int index);

  virtual void onZoom1(bool checked);
  virtual void onZoom1_2(bool checked);

  virtual void onDynamicRange(bool checked);
  virtual void onDynamicRange_2(bool checked);

  virtual void onOpeningEnabling(bool checked);
  virtual void onOpeningIterationChange(int value);
  virtual void onOpeningSizeChange(int value);

  virtual void onClosingEnabling(bool checked);
  virtual void onClosingIterationChange(int value);
  virtual void onClosingSizeChange(int value);

  virtual void onHsvSliderEnabling(bool checked);
  virtual void onHsvSliderChange(int value);

protected:

  virtual void callbackImage(const sensor_msgs::Image::ConstPtr& msg);
  virtual void callbackImage_2(const sensor_msgs::Image::ConstPtr& msg);

  Ui::FtGuiWidget ui_;

  QWidget* widget_;

  // Image viewer 1
  image_transport::Subscriber subscriber_;

  QImage qimage_;
  QMutex qimage_mutex_;

  cv::Mat conversion_mat_;

  // Image viewer 2
  image_transport::Subscriber subscriber_2_;

  QImage qimage_2_;
  QMutex qimage_mutex_2_;

  cv::Mat conversion_mat_2_;
};

}

#endif // rqt_ft_gui__FtGui_H