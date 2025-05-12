// Copyright 2023 Georg Novotny
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <iomanip>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rcpputils/filesystem_helper.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/yaml_config_reader.hpp>
#include <rviz_default_plugins/displays/marker/marker_common.hpp>
#include <rviz_default_plugins/displays/marker_array/marker_array_display.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>

#include <vision_msgs/msg/bounding_box3_d.hpp>
#include <vision_msgs/msg/detection3_d_with_pcl.hpp>
#include <vision_msgs/msg/detection3_d_with_pcl_array.hpp>

// new for pcl vis
#include <rviz_default_plugins/displays/pointcloud/point_cloud2_display.hpp>
#include <rviz_default_plugins/displays/pointcloud/point_cloud_common.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

typedef std::shared_ptr<rviz_rendering::BillboardLine> BillboardLinePtr;

namespace rviz_plugins {
template<class MessageType>
class Detection3DWithPclCommon : public rviz_common::RosTopicDisplay<MessageType>
{
public:
    using MarkerCommon            = rviz_default_plugins::displays::MarkerCommon;
    using Marker                  = visualization_msgs::msg::Marker;
    using BoundingBox3D           = vision_msgs::msg::BoundingBox3D;
    using Detection3DWithPclArray = vision_msgs::msg::Detection3DWithPclArray;

    Detection3DWithPclCommon()
        : rviz_common::RosTopicDisplay<MessageType>()
        , line_width(0.05)
        , alpha()
        , m_marker_common(std::make_unique<MarkerCommon>(this))
        , m_point_cloud_common(std::make_unique<rviz_default_plugins::PointCloudCommon>(this))
        , color_config_path_("")
    { }
    ~Detection3DWithPclCommon() { }

protected:
    float                                                               line_width, alpha;
    std::unique_ptr<MarkerCommon>                                       m_marker_common;
    std::unique_ptr<rviz_default_plugins::PointCloudCommon>             m_point_cloud_common;
    std::vector<BillboardLinePtr>                                       edges_;
    std::string                                                         color_config_path_;
    rviz_common::properties::StringProperty*                            string_property_;
    std::unordered_map<int, visualization_msgs::msg::Marker::SharedPtr> score_markers;

    std::map<std::string, QColor> idToColorMap = {{"car", QColor(255, 165, 0)},
                                                  {"person", QColor(0, 0, 255)},
                                                  {"cyclist", QColor(255, 255, 0)},
                                                  {"motorcycle", QColor(230, 230, 250)}};

    visualization_msgs::msg::Marker::SharedPtr get_marker(const vision_msgs::msg::BoundingBox3D& box)
    {
        auto marker = std::make_shared<Marker>();

        marker->type   = Marker::CUBE;
        marker->action = Marker::ADD;

        marker->pose.position.x    = static_cast<double>(box.center.position.x);
        marker->pose.position.y    = static_cast<double>(box.center.position.y);
        marker->pose.position.z    = static_cast<double>(box.center.position.z);
        marker->pose.orientation.x = static_cast<double>(box.center.orientation.x);
        marker->pose.orientation.y = static_cast<double>(box.center.orientation.y);
        marker->pose.orientation.z = static_cast<double>(box.center.orientation.z);
        marker->pose.orientation.w = static_cast<double>(box.center.orientation.w);

        if (box.size.x < 0.0 || box.size.y < 0.0 || box.size.z < 0.0)
        {
            std::ostringstream oss;
            oss << "Error received BoundingBox3D message with size value less than zero.\n";
            oss << "X: " << box.size.x << " Y: " << box.size.y << " Z: " << box.size.z;
            RVIZ_COMMON_LOG_ERROR_STREAM(oss.str());
            this->setStatus(rviz_common::properties::StatusProperty::Error, "Scale", QString::fromStdString(oss.str()));
        }

        // Some systems can return BoundingBox3D messages with one dimension set to zero.
        // (for example Isaac Sim can have a mesh that is only a plane)
        // This is not supported by Rviz markers so set the scale to a small value if this happens.
        if (box.size.x < 1e-4)
        {
            marker->scale.x = 1e-4;
        }
        else
        {
            marker->scale.x = static_cast<double>(box.size.x);
        }
        if (box.size.y < 1e-4)
        {
            marker->scale.y = 1e-4;
        }
        else
        {
            marker->scale.y = static_cast<double>(box.size.y);
        }
        if (box.size.z < 1e-4)
        {
            marker->scale.z = 1e-4;
        }
        else
        {
            marker->scale.z = static_cast<double>(box.size.z);
        }

        return marker;
    }

    QColor getColor(std::string id = "") const
    {
        QColor color;
        if (id == "")
        {
            color.setRgb(255, 22, 80, 255);
        }
        else
        {
            std::for_each(id.begin(), id.end(), [&](char& c) { c = std::tolower(c, std::locale()); });
            std::string lowercaseId = id;
            auto        it          = idToColorMap.find(lowercaseId);
            if (it != idToColorMap.end())
            {
                color = it->second;
            }
            else
            {
                color.setRgb(190, 190, 190);
            }
        }
        return color;
    }

    // Visualization for PointCloud2
    void showPointCloud(const vision_msgs::msg::Detection3DWithPclArray::ConstSharedPtr& msg, bool pub_empty_msg = false)
    {
        if (!msg)
        {
            RVIZ_COMMON_LOG_ERROR("Received null PointCloud2 message.");
            return;
        }

        if (pub_empty_msg)
        {
            return;
            auto empty_cloud_ptr    = std::make_shared<sensor_msgs::msg::PointCloud2>();
            empty_cloud_ptr->header = msg->header;

            m_point_cloud_common->addMessage(empty_cloud_ptr);
        }
        else
        {
            auto merged_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
            bool got_header = false;
            bool header_mismatch = false;
            

            for (auto& det_and_pcls : msg->detections_and_pcls)
            {
                if(got_header)
                {
                    if(merged_cloud->header.frame_id != det_and_pcls.source_cloud.header.frame_id)
                    {
                        header_mismatch = true;
                        continue;
                    }
                }

                merged_cloud->header = det_and_pcls.source_cloud.header;
                merged_cloud->point_step = det_and_pcls.source_cloud.point_step;
                merged_cloud->row_step = det_and_pcls.source_cloud.row_step;
                merged_cloud->fields = det_and_pcls.source_cloud.fields;
                merged_cloud->is_bigendian = det_and_pcls.source_cloud.is_bigendian;
                got_header = true;

                merged_cloud->data.insert(merged_cloud->data.end(), det_and_pcls.source_cloud.data.begin(), det_and_pcls.source_cloud.data.end());
            }

            if(header_mismatch)
            {
                RVIZ_COMMON_LOG_ERROR("PointCloud2 frame id mismatch in Detection3DWithPclArray message.");
                return;
            }
            
            merged_cloud->height = 1;
            merged_cloud->is_dense = false;
            merged_cloud->width = merged_cloud->data.size() / merged_cloud->point_step;
            
            m_point_cloud_common->addMessage(merged_cloud);
        }
    }

    void showPointCloud(const vision_msgs::msg::Detection3DWithPcl::ConstSharedPtr& msg, bool pub_empty_msg = false)
    {
        if (!msg)
        {
            RVIZ_COMMON_LOG_ERROR("Received null PointCloud2 message.");
            return;
        }

        if (pub_empty_msg)
        {
            return;
            auto empty_cloud_ptr    = std::make_shared<sensor_msgs::msg::PointCloud2>();
            empty_cloud_ptr->header = msg->header;

            m_point_cloud_common->addMessage(empty_cloud_ptr);
        }
        else
        {
            auto source_cloud_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(msg->source_cloud);
            m_point_cloud_common->addMessage(source_cloud_ptr);
        }
    }


    void showBoxes(const vision_msgs::msg::Detection3DWithPclArray::ConstSharedPtr& msg, const bool show_score)
    {
        edges_.clear();
        m_marker_common->clearMarkers();
        ClearScores(show_score);

        for (size_t idx = 0U; idx < msg->detections_and_pcls.size(); idx++)
        {
            const auto marker_ptr = get_marker(msg->detections_and_pcls[idx].bbox);
            QColor     color;
            if (msg->detections_and_pcls[idx].results.size() > 0)
            {
                auto        iter                      = std::max_element(msg->detections_and_pcls[idx].results.begin(),
                                             msg->detections_and_pcls[idx].results.end(),
                                             [](const auto& a, const auto& b) { return a.hypothesis.score < b.hypothesis.score; });
                const auto& result_with_highest_score = *iter;
                color                                 = getColor(result_with_highest_score.hypothesis.class_id);
                if (show_score)
                {
                    ShowScore(msg->detections_and_pcls[idx], result_with_highest_score.hypothesis.score, idx);
                }
            }
            else
            {
                color = getColor(msg->detections_and_pcls[idx].results[0].hypothesis.class_id);
            }
            marker_ptr->color.r = color.red() / 255.0;
            marker_ptr->color.g = color.green() / 255.0;
            marker_ptr->color.b = color.blue() / 255.0;
            marker_ptr->color.a = alpha;
            marker_ptr->ns      = "bounding_box";
            marker_ptr->header  = msg->header;
            marker_ptr->id      = idx;
            m_marker_common->addMessage(marker_ptr);
        }
    }

    void showBoxes(const vision_msgs::msg::Detection3DWithPcl::ConstSharedPtr& msg, const bool show_score)
    {
        edges_.clear();
        m_marker_common->clearMarkers();
        ClearScores(show_score);

        const auto marker_ptr = get_marker(msg->bbox);
        QColor     color;
        if (msg->results.size() > 0)
        {
            auto        iter                      = std::max_element(msg->results.begin(),
                                         msg->results.end(),
                                         [](const auto& a, const auto& b) { return a.hypothesis.score < b.hypothesis.score; });
            const auto& result_with_highest_score = *iter;
            color                                 = getColor(result_with_highest_score.hypothesis.class_id);
            if (show_score)
            {
                ShowScore(*msg, result_with_highest_score.hypothesis.score, 0);
            }
        }
        else
        {
            color = getColor(msg->results[0].hypothesis.class_id);
        }
        marker_ptr->color.r = color.red() / 255.0;
        marker_ptr->color.g = color.green() / 255.0;
        marker_ptr->color.b = color.blue() / 255.0;
        marker_ptr->color.a = alpha;
        marker_ptr->ns      = "bounding_box";
        marker_ptr->header  = msg->header;
        marker_ptr->id      = 0;
        m_marker_common->addMessage(marker_ptr);
    }

    void allocateBillboardLines(size_t num)
    {
        if (num > edges_.size())
        {
            for (size_t i = edges_.size(); i < num; i++)
            {
                BillboardLinePtr line(new rviz_rendering::BillboardLine(this->context_->getSceneManager(), this->scene_node_));
                edges_.push_back(line);
            }
        }
        else if (num < edges_.size())
        {
            edges_.resize(num);
        }
    }

    void showEdges(const vision_msgs::msg::Detection3DWithPclArray::ConstSharedPtr& msg, const bool show_score)
    {
        m_marker_common->clearMarkers();
        ClearScores(show_score);

        allocateBillboardLines(msg->detections_and_pcls.size());

        for (size_t idx = 0; idx < msg->detections_and_pcls.size(); idx++)
        {
            vision_msgs::msg::BoundingBox3D box   = msg->detections_and_pcls[idx].bbox;
            QColor                          color = getColor(msg->detections_and_pcls[idx].results[0].hypothesis.class_id);
            if (msg->detections_and_pcls[idx].results.size() > 0)
            {
                auto iter = std::max_element(msg->detections_and_pcls[idx].results.begin(),
                                             msg->detections_and_pcls[idx].results.end(),
                                             [](const auto& a, const auto& b) { return a.hypothesis.score < b.hypothesis.score; });
                color     = getColor(iter->hypothesis.class_id);
                if (show_score)
                {
                    ShowScore(msg->detections_and_pcls[idx], iter->hypothesis.score, idx);
                }
            }
            geometry_msgs::msg::Vector3 dimensions = box.size;

            BillboardLinePtr edge = edges_[idx];
            edge->clear();
            Ogre::Vector3    position;
            Ogre::Quaternion quaternion;
            if (!this->context_->getFrameManager()->transform(msg->header, box.center, position, quaternion))
            {
                std::ostringstream oss;
                oss << "Error transforming pose";
                oss << " from frame '" << msg->header.frame_id << "'";
                oss << " to frame '" << qPrintable(this->fixed_frame_) << "'";
                RVIZ_COMMON_LOG_ERROR_STREAM(oss.str());
                this->setStatus(rviz_common::properties::StatusProperty::Error, "Transform", QString::fromStdString(oss.str()));
            }
            edge->setPosition(position);
            edge->setOrientation(quaternion);

            edge->setMaxPointsPerLine(2);
            edge->setNumLines(12);
            edge->setLineWidth(line_width);
            edge->setColor(color.red() / 255.0, color.green() / 255.0, color.blue() / 255.0, alpha);

            Ogre::Vector3 A, B, C, D, E, F, G, H;
            A[0] = dimensions.x / 2.0;
            A[1] = dimensions.y / 2.0;
            A[2] = dimensions.z / 2.0;
            B[0] = -dimensions.x / 2.0;
            B[1] = dimensions.y / 2.0;
            B[2] = dimensions.z / 2.0;
            C[0] = -dimensions.x / 2.0;
            C[1] = -dimensions.y / 2.0;
            C[2] = dimensions.z / 2.0;
            D[0] = dimensions.x / 2.0;
            D[1] = -dimensions.y / 2.0;
            D[2] = dimensions.z / 2.0;

            E[0] = dimensions.x / 2.0;
            E[1] = dimensions.y / 2.0;
            E[2] = -dimensions.z / 2.0;
            F[0] = -dimensions.x / 2.0;
            F[1] = dimensions.y / 2.0;
            F[2] = -dimensions.z / 2.0;
            G[0] = -dimensions.x / 2.0;
            G[1] = -dimensions.y / 2.0;
            G[2] = -dimensions.z / 2.0;
            H[0] = dimensions.x / 2.0;
            H[1] = -dimensions.y / 2.0;
            H[2] = -dimensions.z / 2.0;

            edge->addPoint(A);
            edge->addPoint(B);
            edge->finishLine();
            edge->addPoint(B);
            edge->addPoint(C);
            edge->finishLine();
            edge->addPoint(C);
            edge->addPoint(D);
            edge->finishLine();
            edge->addPoint(D);
            edge->addPoint(A);
            edge->finishLine();
            edge->addPoint(E);
            edge->addPoint(F);
            edge->finishLine();
            edge->addPoint(F);
            edge->addPoint(G);
            edge->finishLine();
            edge->addPoint(G);
            edge->addPoint(H);
            edge->finishLine();
            edge->addPoint(H);
            edge->addPoint(E);
            edge->finishLine();
            edge->addPoint(A);
            edge->addPoint(E);
            edge->finishLine();
            edge->addPoint(B);
            edge->addPoint(F);
            edge->finishLine();
            edge->addPoint(C);
            edge->addPoint(G);
            edge->finishLine();
            edge->addPoint(D);
            edge->addPoint(H);
        }
    }

    void showEdges(const vision_msgs::msg::Detection3DWithPcl::ConstSharedPtr& msg, const bool show_score)
    {
        m_marker_common->clearMarkers();
        ClearScores(show_score);

        allocateBillboardLines(1);

        QColor color;
        if (msg->results.size() > 0)
        {
            auto iter = std::max_element(msg->results.begin(),
                                         msg->results.end(),
                                         [](const auto& a, const auto& b) { return a.hypothesis.score < b.hypothesis.score; });
            color     = getColor(iter->hypothesis.class_id);
            if (show_score)
            {
                ShowScore(*msg, iter->hypothesis.score, 0);
            }
        }
        else
        {
            color = getColor(msg->results[0].hypothesis.class_id);
        }
        geometry_msgs::msg::Vector3 dimensions = msg->bbox.size;

        BillboardLinePtr edge = edges_[0];
        edge->clear();
        Ogre::Vector3    position;
        Ogre::Quaternion quaternion;
        if (!this->context_->getFrameManager()->transform(msg->header, msg->bbox.center, position, quaternion))
        {
            std::ostringstream oss;
            oss << "Error transforming pose";
            oss << " from frame '" << msg->header.frame_id << "'";
            oss << " to frame '" << qPrintable(this->fixed_frame_) << "'";
            RVIZ_COMMON_LOG_ERROR_STREAM(oss.str());
            this->setStatus(rviz_common::properties::StatusProperty::Error, "Transform", QString::fromStdString(oss.str()));
            return;
        }
        edge->setPosition(position);
        edge->setOrientation(quaternion);

        edge->setMaxPointsPerLine(2);
        edge->setNumLines(12);
        edge->setLineWidth(line_width);
        edge->setColor(color.red() / 255.0, color.green() / 255.0, color.blue() / 255.0, alpha);

        Ogre::Vector3 A, B, C, D, E, F, G, H;
        A[0] = dimensions.x / 2.0;
        A[1] = dimensions.y / 2.0;
        A[2] = dimensions.z / 2.0;
        B[0] = -dimensions.x / 2.0;
        B[1] = dimensions.y / 2.0;
        B[2] = dimensions.z / 2.0;
        C[0] = -dimensions.x / 2.0;
        C[1] = -dimensions.y / 2.0;
        C[2] = dimensions.z / 2.0;
        D[0] = dimensions.x / 2.0;
        D[1] = -dimensions.y / 2.0;
        D[2] = dimensions.z / 2.0;

        E[0] = dimensions.x / 2.0;
        E[1] = dimensions.y / 2.0;
        E[2] = -dimensions.z / 2.0;
        F[0] = -dimensions.x / 2.0;
        F[1] = dimensions.y / 2.0;
        F[2] = -dimensions.z / 2.0;
        G[0] = -dimensions.x / 2.0;
        G[1] = -dimensions.y / 2.0;
        G[2] = -dimensions.z / 2.0;
        H[0] = dimensions.x / 2.0;
        H[1] = -dimensions.y / 2.0;
        H[2] = -dimensions.z / 2.0;

        edge->addPoint(A);
        edge->addPoint(B);
        edge->finishLine();
        edge->addPoint(B);
        edge->addPoint(C);
        edge->finishLine();
        edge->addPoint(C);
        edge->addPoint(D);
        edge->finishLine();
        edge->addPoint(D);
        edge->addPoint(A);
        edge->finishLine();
        edge->addPoint(E);
        edge->addPoint(F);
        edge->finishLine();
        edge->addPoint(F);
        edge->addPoint(G);
        edge->finishLine();
        edge->addPoint(G);
        edge->addPoint(H);
        edge->finishLine();
        edge->addPoint(H);
        edge->addPoint(E);
        edge->finishLine();
        edge->addPoint(A);
        edge->addPoint(E);
        edge->finishLine();
        edge->addPoint(B);
        edge->addPoint(F);
        edge->finishLine();
        edge->addPoint(C);
        edge->addPoint(G);
        edge->finishLine();
        edge->addPoint(D);
        edge->addPoint(H);
    }

    void ShowScore(const vision_msgs::msg::Detection3DWithPcl detection, const double score, const size_t idx)
    {
        auto marker    = std::make_shared<Marker>();
        marker->type   = Marker::TEXT_VIEW_FACING;
        marker->action = Marker::ADD;
        marker->header = detection.header;
        std::ostringstream oss;
        oss << std::fixed;
        oss << std::setprecision(2);
        oss << score;
        marker->text            = oss.str();
        marker->scale.z         = 0.5;    // Set the size of the text
        marker->id              = idx;
        marker->ns              = "score";
        marker->color.r         = 1.0f;
        marker->color.g         = 1.0f;
        marker->color.b         = 1.0f;
        marker->color.a         = alpha;
        marker->pose.position.x = static_cast<double>(detection.bbox.center.position.x);
        marker->pose.position.y = static_cast<double>(detection.bbox.center.position.y);
        marker->pose.position.z = static_cast<double>(detection.bbox.center.position.z + (detection.bbox.size.z / 2.0) * 1.2);

        // Add the marker to the MarkerArray message
        m_marker_common->addMessage(marker);
        score_markers[idx] = marker;
    }

    void ClearScores(const bool show_score)
    {
        if (!show_score)
        {
            for (auto& [id, marker] : score_markers)
            {
                marker->action = visualization_msgs::msg::Marker::DELETE;
                m_marker_common->addMessage(marker);
            }
            score_markers.clear();
        }
    }

    void updateColorConfig()
    {
        std::ostringstream oss;
        const std::string  tmp_path = string_property_->getStdString();
        if (rcpputils::fs::exists(tmp_path))
        {
            color_config_path_ = tmp_path;
            std::ifstream fin(color_config_path_);
            YAML::Node    config = YAML::Load(fin);
            // Iterate through the YAML file
            for (YAML::const_iterator it = config.begin(); it != config.end(); ++it)
            {
                std::string key = it->first.as<std::string>();
                int         r   = it->second["r"].as<int>();
                int         g   = it->second["g"].as<int>();
                int         b   = it->second["b"].as<int>();
                // Create a QColor object with the RGB values
                QColor color = QColor(r, g, b);

                // Insert the key-value pair into the map
                idToColorMap[key] = color;
            }
            this->setStatus(rviz_common::properties::StatusProperty::Ok, "Config File", QString::fromStdString(oss.str()));
        }
        else
        {
            oss << " File: '" << string_property_->getStdString() << "' does not exist.";
            RVIZ_COMMON_LOG_ERROR_STREAM(oss.str());
            this->setStatus(rviz_common::properties::StatusProperty::Error, "Config File", QString::fromStdString(oss.str()));
        }
    }
};
}    // namespace rviz_plugins
