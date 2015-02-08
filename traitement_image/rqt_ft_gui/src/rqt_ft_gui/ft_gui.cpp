#include <rqt_ft_gui/ft_gui.h>

#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/highgui/highgui.hpp>

#include <QMessageBox>
#include <QPainter>

namespace rqt_ft_gui {

FtGui::FtGui()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  setObjectName("FtGui");
}

void FtGui::initPlugin(qt_gui_cpp::PluginContext& context)
{
  widget_ = new QWidget();
  ui_.setupUi(widget_);

  if (context.serialNumber() > 1)
  {
    widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);


  // Init image viewer 1
  ui_.image_frame->installEventFilter(this);

  updateTopicList();
  ui_.topics_combo_box->setCurrentIndex(ui_.topics_combo_box->findText(""));
  connect(ui_.topics_combo_box, SIGNAL(currentIndexChanged(int)), this, SLOT(onTopicChanged(int)));

  ui_.refresh_topics_push_button->setIcon(QIcon::fromTheme("view-refresh"));
  connect(ui_.refresh_topics_push_button, SIGNAL(pressed()), this, SLOT(updateTopicList()));

  ui_.zoom_1_push_button->setIcon(QIcon::fromTheme("zoom-original"));
  connect(ui_.zoom_1_push_button, SIGNAL(toggled(bool)), this, SLOT(onZoom1(bool)));
  
  connect(ui_.dynamic_range_check_box, SIGNAL(toggled(bool)), this, SLOT(onDynamicRange(bool)));

  // Init image viewer 2
  ui_.image_frame_2->installEventFilter(this);

  updateTopicList_2();
  ui_.topics_combo_box_2->setCurrentIndex(ui_.topics_combo_box_2->findText(""));
  connect(ui_.topics_combo_box_2, SIGNAL(currentIndexChanged(int)), this, SLOT(onTopicChanged_2(int)));

  ui_.refresh_topics_push_button_2->setIcon(QIcon::fromTheme("view-refresh"));
  connect(ui_.refresh_topics_push_button_2, SIGNAL(pressed()), this, SLOT(updateTopicList_2()));

  ui_.zoom_1_push_button_2->setIcon(QIcon::fromTheme("zoom-original"));
  connect(ui_.zoom_1_push_button_2, SIGNAL(toggled(bool)), this, SLOT(onZoom1_2(bool)));
  
  connect(ui_.dynamic_range_check_box_2, SIGNAL(toggled(bool)), this, SLOT(onDynamicRange_2(bool)));
}

bool FtGui::eventFilter(QObject* watched, QEvent* event)
{
  if (watched == ui_.image_frame && event->type() == QEvent::Paint)
  {
    QPainter painter(ui_.image_frame);
    if (!qimage_.isNull())
    {
      ui_.image_frame->resizeToFitAspectRatio();
      // TODO: check if full draw is really necessary
      //QPaintEvent* paint_event = dynamic_cast<QPaintEvent*>(event);
      //painter.drawImage(paint_event->rect(), qimage_);
      qimage_mutex_.lock();
      painter.drawImage(ui_.image_frame->contentsRect(), qimage_);
      qimage_mutex_.unlock();
    } else {
      // default image with gradient
      QLinearGradient gradient(0, 0, ui_.image_frame->frameRect().width(), ui_.image_frame->frameRect().height());
      gradient.setColorAt(0, Qt::white);
      gradient.setColorAt(1, Qt::black);
      painter.setBrush(gradient);
      painter.drawRect(0, 0, ui_.image_frame->frameRect().width() + 1, ui_.image_frame->frameRect().height() + 1);
    }
    return false;
  }
  else if (watched == ui_.image_frame_2 && event->type() == QEvent::Paint)
  {
    QPainter painter(ui_.image_frame_2);
    if (!qimage_2_.isNull())
    {
      ui_.image_frame_2->resizeToFitAspectRatio();
      // TODO: check if full draw is really necessary
      //QPaintEvent* paint_event = dynamic_cast<QPaintEvent*>(event);
      //painter.drawImage(paint_event->rect(), qimage_2_);
      qimage_mutex_2_.lock();
      painter.drawImage(ui_.image_frame_2->contentsRect(), qimage_2_);
      qimage_mutex_2_.unlock();
    } else {
      // default image with gradient
      QLinearGradient gradient(0, 0, ui_.image_frame_2->frameRect().width(), ui_.image_frame_2->frameRect().height());
      gradient.setColorAt(0, Qt::white);
      gradient.setColorAt(1, Qt::black);
      painter.setBrush(gradient);
      painter.drawRect(0, 0, ui_.image_frame_2->frameRect().width() + 1, ui_.image_frame_2->frameRect().height() + 1);
    }
    return false;
  }

  return QObject::eventFilter(watched, event);
}

void FtGui::shutdownPlugin()
{
  subscriber_.shutdown();
  subscriber_2_.shutdown();
}

void FtGui::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // Image viewer 1
  QString topic = ui_.topics_combo_box->currentText();
  //qDebug("FtGui::saveSettings() topic '%s'", topic.toStdString().c_str());
  instance_settings.setValue("topic", topic);
  instance_settings.setValue("zoom1", ui_.zoom_1_push_button->isChecked());
  instance_settings.setValue("dynamic_range", ui_.dynamic_range_check_box->isChecked());
  instance_settings.setValue("max_range", ui_.max_range_double_spin_box->value());

  // Image viewer 2
  QString topic_2 = ui_.topics_combo_box_2->currentText();
  //qDebug("FtGui::saveSettings() topic '%s'", topic.toStdString().c_str());
  instance_settings.setValue("topic_2", topic_2);
  instance_settings.setValue("zoom1_2", ui_.zoom_1_push_button_2->isChecked());
  instance_settings.setValue("dynamic_range_2", ui_.dynamic_range_check_box_2->isChecked());
  instance_settings.setValue("max_range_2", ui_.max_range_double_spin_box_2->value());
}

void FtGui::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // Image viewer 1
  bool zoom1_checked = instance_settings.value("zoom1", false).toBool();
  ui_.zoom_1_push_button->setChecked(zoom1_checked);

  bool dynamic_range_checked = instance_settings.value("dynamic_range", false).toBool();
  ui_.dynamic_range_check_box->setChecked(dynamic_range_checked);

  double max_range = instance_settings.value("max_range", ui_.max_range_double_spin_box->value()).toDouble();
  ui_.max_range_double_spin_box->setValue(max_range);

  QString topic = instance_settings.value("topic", "").toString();
  //qDebug("FtGui::restoreSettings() topic '%s'", topic.toStdString().c_str());
  selectTopic(topic);

  // Image viewer 2
  bool zoom1_checked_2 = instance_settings.value("zoom1_2", false).toBool();
  ui_.zoom_1_push_button_2->setChecked(zoom1_checked_2);

  bool dynamic_range_checked_2 = instance_settings.value("dynamic_range_2", false).toBool();
  ui_.dynamic_range_check_box_2->setChecked(dynamic_range_checked_2);

  double max_range_2 = instance_settings.value("max_range_2", ui_.max_range_double_spin_box_2->value()).toDouble();
  ui_.max_range_double_spin_box_2->setValue(max_range_2);

  QString topic_2 = instance_settings.value("topic_2", "").toString();
  //qDebug("FtGui::restoreSettings() topic '%s'", topic.toStdString().c_str());
  selectTopic_2(topic_2);
}

void FtGui::updateTopicList()
{
  QSet<QString> message_types;
  message_types.insert("sensor_msgs/Image");

  // get declared transports
  QList<QString> transports;
  image_transport::ImageTransport it(getNodeHandle());
  std::vector<std::string> declared = it.getDeclaredTransports();
  for (std::vector<std::string>::const_iterator it = declared.begin(); it != declared.end(); it++)
  {
    //qDebug("FtGui::updateTopicList() declared transport '%s'", it->c_str());
    QString transport = it->c_str();

    // strip prefix from transport name
    QString prefix = "image_transport/";
    if (transport.startsWith(prefix))
    {
      transport = transport.mid(prefix.length());
    }
    transports.append(transport);
  }

  QString selected = ui_.topics_combo_box->currentText();

  // fill combo box
  QList<QString> topics = getTopicList(message_types, transports);
  topics.append("");
  qSort(topics);
  ui_.topics_combo_box->clear();
  for (QList<QString>::const_iterator it = topics.begin(); it != topics.end(); it++)
  {
    QString label(*it);
    label.replace(" ", "/");
    ui_.topics_combo_box->addItem(label, QVariant(*it));
  }

  // restore previous selection
  selectTopic(selected);
}

void FtGui::updateTopicList_2()
{
  QSet<QString> message_types;
  message_types.insert("sensor_msgs/Image");

  // get declared transports
  QList<QString> transports;
  image_transport::ImageTransport it(getNodeHandle());
  std::vector<std::string> declared = it.getDeclaredTransports();
  for (std::vector<std::string>::const_iterator it = declared.begin(); it != declared.end(); it++)
  {
    //qDebug("FtGui::updateTopicList() declared transport '%s'", it->c_str());
    QString transport = it->c_str();

    // strip prefix from transport name
    QString prefix = "image_transport/";
    if (transport.startsWith(prefix))
    {
      transport = transport.mid(prefix.length());
    }
    transports.append(transport);
  }

  QString selected = ui_.topics_combo_box_2->currentText();

  // fill combo box
  QList<QString> topics = getTopicList(message_types, transports);
  topics.append("");
  qSort(topics);
  ui_.topics_combo_box_2->clear();
  for (QList<QString>::const_iterator it = topics.begin(); it != topics.end(); it++)
  {
    QString label(*it);
    label.replace(" ", "/");
    ui_.topics_combo_box_2->addItem(label, QVariant(*it));
  }

  // restore previous selection
  selectTopic_2(selected);
}

QList<QString> FtGui::getTopicList(const QSet<QString>& message_types, const QList<QString>& transports)
{
  ros::master::V_TopicInfo topic_info;
  ros::master::getTopics(topic_info);

  QSet<QString> all_topics;
  for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
  {
    all_topics.insert(it->name.c_str());
  }

  QList<QString> topics;
  for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
  {
    if (message_types.contains(it->datatype.c_str()))
    {
      QString topic = it->name.c_str();

      // add raw topic
      topics.append(topic);
      //qDebug("FtGui::getTopicList() raw topic '%s'", topic.toStdString().c_str());
      
      // add transport specific sub-topics
      for (QList<QString>::const_iterator jt = transports.begin(); jt != transports.end(); jt++)
      {
        if (all_topics.contains(topic + "/" + *jt))
        {
          QString sub = topic + " " + *jt;
          topics.append(sub);
          //qDebug("FtGui::getTopicList() transport specific sub-topic '%s'", sub.toStdString().c_str());
        }
      }
    }
  }
  return topics;
}

void FtGui::selectTopic(const QString& topic)
{
  int index = ui_.topics_combo_box->findText(topic);
  if (index == -1)
  {
    index = ui_.topics_combo_box->findText("");
  }
  ui_.topics_combo_box->setCurrentIndex(index);
}

void FtGui::selectTopic_2(const QString& topic)
{
  int index = ui_.topics_combo_box_2->findText(topic);
  if (index == -1)
  {
    index = ui_.topics_combo_box_2->findText("");
  }
  ui_.topics_combo_box_2->setCurrentIndex(index);
}

void FtGui::onTopicChanged(int index)
{
  subscriber_.shutdown();

  // reset image on topic change
  qimage_ = QImage();
  ui_.image_frame->update();

  QStringList parts = ui_.topics_combo_box->itemData(index).toString().split(" ");
  QString topic = parts.first();
  QString transport = parts.length() == 2 ? parts.last() : "raw";

  if (!topic.isEmpty())
  {
    image_transport::ImageTransport it(getNodeHandle());
    image_transport::TransportHints hints(transport.toStdString());
    try {
      subscriber_ = it.subscribe(topic.toStdString(), 1, &FtGui::callbackImage, this, hints);
      //qDebug("FtGui::onTopicChanged() to topic '%s' with transport '%s'", topic.toStdString().c_str(), subscriber_.getTransport().c_str());
    } catch (image_transport::TransportLoadException& e) {
      QMessageBox::warning(widget_, tr("Loading image transport plugin failed"), e.what());
    }
  }
}

void FtGui::onTopicChanged_2(int index)
{
  subscriber_2_.shutdown();

  // reset image on topic change
  qimage_2_ = QImage();
  ui_.image_frame_2->update();

  QStringList parts = ui_.topics_combo_box_2->itemData(index).toString().split(" ");
  QString topic = parts.first();
  QString transport = parts.length() == 2 ? parts.last() : "raw";

  if (!topic.isEmpty())
  {
    image_transport::ImageTransport it(getNodeHandle());
    image_transport::TransportHints hints(transport.toStdString());
    try {
      subscriber_2_ = it.subscribe(topic.toStdString(), 1, &FtGui::callbackImage_2, this, hints);
      //qDebug("FtGui::onTopicChanged_2() to topic '%s' with transport '%s'", topic.toStdString().c_str(), subscriber_2_.getTransport().c_str());
    } catch (image_transport::TransportLoadException& e) {
      QMessageBox::warning(widget_, tr("Loading image transport plugin failed"), e.what());
    }
  }
}

void FtGui::onZoom1(bool checked)
{
  if (checked)
  {
    if (qimage_.isNull())
    {
      return;
    }
    ui_.image_frame->setInnerFrameFixedSize(qimage_.size());
    widget_->resize(ui_.image_frame->size());
    widget_->setMinimumSize(widget_->sizeHint());
    widget_->setMaximumSize(widget_->sizeHint());
  } else {
    ui_.image_frame->setInnerFrameMinimumSize(QSize(80, 60));
    ui_.image_frame->setMaximumSize(QSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX));
    widget_->setMinimumSize(QSize(80, 60));
    widget_->setMaximumSize(QSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX));
  }
}

void FtGui::onZoom1_2(bool checked)
{
  if (checked)
  {
    if (qimage_2_.isNull())
    {
      return;
    }
    ui_.image_frame_2->setInnerFrameFixedSize(qimage_2_.size());
    widget_->resize(ui_.image_frame_2->size());
    widget_->setMinimumSize(widget_->sizeHint());
    widget_->setMaximumSize(widget_->sizeHint());
  } else {
    ui_.image_frame_2->setInnerFrameMinimumSize(QSize(80, 60));
    ui_.image_frame_2->setMaximumSize(QSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX));
    widget_->setMinimumSize(QSize(80, 60));
    widget_->setMaximumSize(QSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX));
  }
}

void FtGui::onDynamicRange(bool checked)
{
  ui_.max_range_double_spin_box->setEnabled(!checked);
}

void FtGui::onDynamicRange_2(bool checked)
{
  ui_.max_range_double_spin_box_2->setEnabled(!checked);
}

void FtGui::callbackImage(const sensor_msgs::Image::ConstPtr& msg)
{
  try
  {
    // First let cv_bridge do its magic (QImage expect BGR)
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    conversion_mat_ = cv_ptr->image;
  }
  catch (cv_bridge::Exception& e)
  {
    // If we're here, there is no conversion that makes sense, but let's try to imagine a few first
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
    if (msg->encoding == "CV_8UC3")
    {
      // assuming it is rgb
      conversion_mat_ = cv_ptr->image;
    } else if (msg->encoding == "8UC1") {
      // convert gray to rgb
      cv::cvtColor(cv_ptr->image, conversion_mat_, CV_GRAY2RGB);
    } else if (msg->encoding == "16UC1" || msg->encoding == "32FC1") {
      // scale / quantify
      double min = 0;
      double max = ui_.max_range_double_spin_box->value();
      if (msg->encoding == "16UC1") max *= 1000;
      if (ui_.dynamic_range_check_box->isChecked())
      {
        // dynamically adjust range based on min/max in image
        cv::minMaxLoc(cv_ptr->image, &min, &max);
        if (min == max) {
          // completely homogeneous images are displayed in gray
          min = 0;
          max = 2;
        }
      }
      cv::Mat img_scaled_8u;
      cv::Mat(cv_ptr->image-min).convertTo(img_scaled_8u, CV_8UC1, 255. / (max - min));
      cv::cvtColor(img_scaled_8u, conversion_mat_, CV_GRAY2RGB);
    } else {
      qWarning("FtGui.callback_image() could not convert image from '%s' to 'rgb8' (%s)", msg->encoding.c_str(), e.what());
      qimage_ = QImage();
      return;
    }
  }

  // copy temporary image as it uses the conversion_mat_ for storage which is asynchronously overwritten in the next callback invocation
  QImage image(conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows, QImage::Format_RGB888);
  qimage_mutex_.lock();
  qimage_ = image.copy();
  qimage_mutex_.unlock();

  ui_.image_frame->setAspectRatio(qimage_.width(), qimage_.height());
  if (!ui_.zoom_1_push_button->isEnabled())
  {
    ui_.zoom_1_push_button->setEnabled(true);
    onZoom1(ui_.zoom_1_push_button->isChecked());
  }
  ui_.image_frame->update();
}

void FtGui::callbackImage_2(const sensor_msgs::Image::ConstPtr& msg)
{
  try
  {
    // First let cv_bridge do its magic (QImage expect BGR)
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    conversion_mat_2_ = cv_ptr->image;
  }
  catch (cv_bridge::Exception& e)
  {
    // If we're here, there is no conversion that makes sense, but let's try to imagine a few first
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
    if (msg->encoding == "CV_8UC3")
    {
      // assuming it is rgb
      conversion_mat_2_ = cv_ptr->image;
    } else if (msg->encoding == "8UC1") {
      // convert gray to rgb
      cv::cvtColor(cv_ptr->image, conversion_mat_2_, CV_GRAY2RGB);
    } else if (msg->encoding == "16UC1" || msg->encoding == "32FC1") {
      // scale / quantify
      double min = 0;
      double max = ui_.max_range_double_spin_box_2->value();
      if (msg->encoding == "16UC1") max *= 1000;
      if (ui_.dynamic_range_check_box_2->isChecked())
      {
        // dynamically adjust range based on min/max in image
        cv::minMaxLoc(cv_ptr->image, &min, &max);
        if (min == max) {
          // completely homogeneous images are displayed in gray
          min = 0;
          max = 2;
        }
      }
      cv::Mat img_scaled_8u;
      cv::Mat(cv_ptr->image-min).convertTo(img_scaled_8u, CV_8UC1, 255. / (max - min));
      cv::cvtColor(img_scaled_8u, conversion_mat_2_, CV_GRAY2RGB);
    } else {
      qWarning("FtGui.callback_image_2() could not convert image from '%s' to 'rgb8' (%s)", msg->encoding.c_str(), e.what());
      qimage_2_ = QImage();
      return;
    }
  }

  // copy temporary image as it uses the conversion_mat_ for storage which is asynchronously overwritten in the next callback invocation
  QImage image(conversion_mat_2_.data, conversion_mat_2_.cols, conversion_mat_2_.rows, QImage::Format_RGB888);
  qimage_mutex_2_.lock();
  qimage_2_ = image.copy();
  qimage_mutex_2_.unlock();

  ui_.image_frame_2->setAspectRatio(qimage_2_.width(), qimage_2_.height());
  if (!ui_.zoom_1_push_button_2->isEnabled())
  {
    ui_.zoom_1_push_button_2->setEnabled(true);
    onZoom1_2(ui_.zoom_1_push_button_2->isChecked());
  }
  ui_.image_frame_2->update();
}

}

PLUGINLIB_EXPORT_CLASS(rqt_ft_gui::FtGui, rqt_gui_cpp::Plugin)