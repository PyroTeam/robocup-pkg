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

protected:

  virtual QList<QString> getTopicList(const QSet<QString>& message_types, const QList<QString>& transports);

  virtual void selectTopic(const QString& topic);

protected slots:

  virtual void onTopicChanged(int index);

  virtual void onZoom1(bool checked);

  virtual void onDynamicRange(bool checked);

protected:

  virtual void callbackImage(const sensor_msgs::Image::ConstPtr& msg);

  Ui::FtGuiWidget ui_;

  QWidget* widget_;

  image_transport::Subscriber subscriber_;

  QImage qimage_;
  QMutex qimage_mutex_;

  cv::Mat conversion_mat_;

};

}

#endif // rqt_ft_gui__FtGui_H