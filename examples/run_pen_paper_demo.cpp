// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

#include <filesystem/filesystem.h>
#include <m3t/generator.h>
#include <m3t/tracker.h>
#include <m3t/loader_camera.h>



/**
 * Class that visualizes the drawing of a stabilo pen on a paper
 */
class DrawingPublisher : public m3t::Publisher {
 private:
  // DIN A4 paper dimensions 210mmx297mm
  static constexpr int kDefaultImageWidth = 210;
  static constexpr int kDefaultImageHeight = 297;
  static constexpr float kA4PaperWidthOffset = 0.105f;
  static constexpr float kA4PaperHeightOffset = 0.1485f;
  static constexpr float kA4PaperDepthOffset = 0.0f;

 public:
  DrawingPublisher(const std::string& name,
                   const std::shared_ptr<m3t::Body>& paper_body_ptr,
                   const std::shared_ptr<m3t::Body>& stabilo_body_ptr,
                   const std::string& window_name = "AR Drawing",
                   float scaling_factor = 2.0f)
      : Publisher{name},
        paper_body_ptr_{paper_body_ptr},
        stabilo_body_ptr_{stabilo_body_ptr},
        window_name_{window_name},
        scaling_factor_{scaling_factor} {}

  void StartSavingImages(const std::filesystem::path& save_directory,
                         const std::string& save_image_type = "png") {
    save_images_ = true;
    save_directory_ = save_directory;
    save_image_type_ = save_image_type;
  }

  void StopSavingImages() { save_images_ = false; }

  bool SetUp() override {
    image_ = cv::Mat{cv::Size2i{int(kDefaultImageWidth * scaling_factor_),
                                int(kDefaultImageHeight * scaling_factor_)},
                     CV_8UC1, 255};
    set_up_ = true;
    return true;
  }

  bool UpdatePublisher(int iteration) override {
    // Calculate pose between tip and paper frame
    m3t::Transform3fA tip2paper_pose{paper_body_ptr_->world2body_pose() *
                                     stabilo_body_ptr_->body2world_pose()};
    Eigen::Vector3f tip2paper_trans{tip2paper_pose.translation()};

    // Draw on image if stabilo is close enough to paper
    if (tip2paper_trans.x() < kA4PaperWidthOffset &&
        tip2paper_trans.x() > -kA4PaperWidthOffset &&
        tip2paper_trans.y() < kA4PaperHeightOffset &&
        tip2paper_trans.y() > -kA4PaperHeightOffset &&
        tip2paper_trans.z() < kA4PaperDepthOffset) {
      float x = (tip2paper_trans.x() + kA4PaperWidthOffset) * 1000.0f *
                scaling_factor_;
      float y = (kA4PaperHeightOffset - tip2paper_trans.y()) * 1000.0f *
                scaling_factor_;
      cv::Point2i coordinate{int(x + 0.5f), int(y + 0.5f)};
      if (writing_) {
        int thickness = int(2.0f * scaling_factor_ + 0.5f);
        cv::line(image_, coordinate, previous_coordinate_, 0, thickness,
                 cv::LINE_4);
      }
      previous_coordinate_ = coordinate;
      writing_ = true;
    } else {
      writing_ = false;
    }

    // Display and save image
    cv::imshow(window_name_, image_);
    if (save_images_) {
      std::filesystem::path path{save_directory_ / (name_ + "_image_" +
                                                    std::to_string(iteration) +
                                                    "." + save_image_type_)};
      cv::imwrite(path.string(), image_);
    }
    return true;
  }

 private:
  // Parameters
  std::shared_ptr<m3t::Body> paper_body_ptr_{};
  std::shared_ptr<m3t::Body> stabilo_body_ptr_{};
  std::string window_name_;
  float scaling_factor_;
  std::filesystem::path save_directory_{};
  std::string save_image_type_ = "png";
  bool save_images_ = false;

  // Internal data
  bool writing_ = false;
  cv::Point2i previous_coordinate_;
  cv::Mat image_;


};


int main(int argc, char* argv[]) {
  const std::filesystem::path configfile_path{
      "C:/Users/Alessandro Rastelli/Desktop/M3T_Protom-master/data/pen_paper_demo/calibro/calibro_config.yaml"};
  // Generate tracker
  std::shared_ptr<m3t::Tracker> tracker_ptr;
  if (!GenerateConfiguredTracker(configfile_path, &tracker_ptr)) return -1;
  if (!tracker_ptr->SetUp()) return -1;
  std::cout << "Ciao, mondo!" << std::endl;  // Stampa una stringa e va a capo

  // Search required bodies
  std::shared_ptr<m3t::Body> paper_body_ptr{};
  std::shared_ptr<m3t::Body> stabilo_body_ptr{};
  std::shared_ptr<m3t::Body> stabilo2_body_ptr{};
  std::shared_ptr<m3t::Body> stabilo3_body_ptr{};


  for (const auto& body_ptr : tracker_ptr->body_ptrs()) {
    if (body_ptr->name() == "calibro") paper_body_ptr = body_ptr;
    if (body_ptr->name() == "calibroA") stabilo_body_ptr = body_ptr;
    if (body_ptr->name() == "calibroB") stabilo2_body_ptr = body_ptr;
    if (body_ptr->name() == "rondella") stabilo3_body_ptr = body_ptr;
   }
  if (!paper_body_ptr || !stabilo_body_ptr) return -1;
   std::cout << "Ciao, rondella!" << std::endl;  // Stampa una stringa e va a capo


 //std::shared_ptr<m3t::Body> bottle_body_ptr{};
 //  for (const auto& body_ptr : tracker_ptr->body_ptrs()) {
 //  if (body_ptr->name() == "bottle") bottle_body_ptr = body_ptr;
 //  }
 std::cout << "Ciao, mondo2!" << std::endl;  // Stampa una stringa e va a capo

  //// Add publisher that updates drawing
  //auto drawing_publisher_ptr{std::make_shared<DrawingPublisher>(
  //    DrawingPublisher("Drawing Publisher", paper_body_ptr, stabilo_body_ptr))};
  //tracker_ptr->AddPublisher(drawing_publisher_ptr);
  std::cout << "Ciao, mondo3!" << std::endl;  // Stampa una stringa e va a capo  

  // Setup and run tracker
  if (!tracker_ptr->SetUp()) return -1;
  if (!tracker_ptr->RunTrackerProcess(true, false)) return 0;
  std::cout << "Ciao, mondo4!" << std::endl;  // Stampa una stringa e va a capo
  return 0;
  
}
