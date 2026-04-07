//
// Created by qiayuanl on 5/14/25.
//

#include "motion_tracking_controller/MotionOnnxPolicy.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

std::string trim(const std::string& value) {
  const auto begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of(" \t\r\n");
  return value.substr(begin, end - begin + 1);
}

std::vector<std::string> split(const std::string& value, const char delimiter) {
  std::vector<std::string> tokens;
  std::stringstream ss(value);
  std::string token;
  while (std::getline(ss, token, delimiter)) {
    tokens.push_back(trim(token));
  }
  return tokens;
}

std::vector<legged::scalar_t> parseNumberList(const std::string& value, const char delimiter, const std::string& fieldName,
                                              size_t lineNumber) {
  std::vector<legged::scalar_t> numbers;
  for (const auto& token : split(value, delimiter)) {
    if (token.empty()) {
      continue;
    }
    try {
      numbers.push_back(static_cast<legged::scalar_t>(std::stod(token)));
    } catch (const std::exception&) {
      throw std::runtime_error("Failed to parse numeric token '" + token + "' in field '" + fieldName + "' at line " +
                               std::to_string(lineNumber) + ".");
    }
  }
  return numbers;
}

}  // namespace

namespace legged {

void MotionOnnxPolicy::reset() {
  OnnxPolicy::reset();
  timeStep_ = startStep_;
  forward(vector_t::Zero(getObservationSize()));
}

vector_t MotionOnnxPolicy::forward(const vector_t& observations) {
  const auto currentStep = timeStep_;
  tensor2d_t timeStep(1, 1);
  timeStep(0, 0) = static_cast<tensor_element_t>(timeStep_++);
  inputTensors_[name2Index_.at("time_step")] = timeStep;
  OnnxPolicy::forward(observations);

  jointPosition_ = outputTensors_[name2Index_.at("joint_pos")].row(0).cast<scalar_t>();
  jointVelocity_ = outputTensors_[name2Index_.at("joint_vel")].row(0).cast<scalar_t>();
  bodyPositions_.clear();
  bodyOrientations_.clear();

  auto body_pos_w = outputTensors_[name2Index_.at("body_pos_w")].cast<scalar_t>();
  auto body_quat_w = outputTensors_[name2Index_.at("body_quat_w")].cast<scalar_t>();

  for (size_t i = 0; i < body_pos_w.rows(); ++i) {
    vector3_t pos = body_pos_w.row(i);
    vector_t quat = body_quat_w.row(i);
    quaternion_t ori;
    ori.w() = quat(0);
    ori.coeffs().head(3) = quat.tail(3);
    bodyPositions_.push_back(pos);
    bodyOrientations_.push_back(ori);
  }

  loadLocalReferenceIfNeeded();
  if (!localReferenceFrames_.empty()) {
    size_t frameIndex = currentStep;
    if (loopLocalReference_) {
      frameIndex %= localReferenceFrames_.size();
    } else if (frameIndex >= localReferenceFrames_.size()) {
      frameIndex = localReferenceFrames_.size() - 1;
    }
    const auto& frame = localReferenceFrames_[frameIndex];
    jointPosition_ = frame.jointPosition;
    jointVelocity_ = frame.jointVelocity;
    bodyPositions_ = frame.bodyPositions;
    bodyOrientations_ = frame.bodyOrientations;
  }

  return getLastAction();
}

void MotionOnnxPolicy::parseMetadata() {
  OnnxPolicy::parseMetadata();
  anchorBodyName_ = getMetadataStr("anchor_body_name");
  std::cout << '\t' << "anchor_body_name: " << anchorBodyName_ << '\n';
  bodyNames_ = parseCsv<std::string>(getMetadataStr("body_names"));
  std::cout << '\t' << "body_names: " << bodyNames_ << '\n';
}

void MotionOnnxPolicy::loadLocalReferenceIfNeeded() {
  if (localReferencePath_.empty() || localReferenceLoaded_) {
    return;
  }
  localReferenceLoaded_ = true;

  std::ifstream file(localReferencePath_);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open local reference file: " + localReferencePath_);
  }

  const auto jointCount = static_cast<size_t>(jointPosition_.size());
  const auto bodyCount = bodyNames_.size();
  std::string rawLine;
  size_t lineNumber = 0;

  while (std::getline(file, rawLine)) {
    ++lineNumber;
    const auto commentPos = rawLine.find('#');
    if (commentPos != std::string::npos) {
      rawLine = rawLine.substr(0, commentPos);
    }
    const auto line = trim(rawLine);
    if (line.empty()) {
      continue;
    }

    const auto fields = split(line, ';');
    if (fields.size() != 4) {
      throw std::runtime_error("Invalid local reference format at line " + std::to_string(lineNumber) +
                               ": expected 4 ';'-separated fields (joint_pos; joint_vel; body_pos; body_quat).");
    }

    const auto jointPosVals = parseNumberList(fields[0], ',', "joint_pos", lineNumber);
    const auto jointVelVals = parseNumberList(fields[1], ',', "joint_vel", lineNumber);
    const auto bodyPosVals = parseNumberList(fields[2], ',', "body_pos", lineNumber);
    const auto bodyQuatVals = parseNumberList(fields[3], ',', "body_quat", lineNumber);

    if (jointPosVals.size() != jointCount) {
      throw std::runtime_error("joint_pos size mismatch at line " + std::to_string(lineNumber) + ": expected " +
                               std::to_string(jointCount) + ", got " + std::to_string(jointPosVals.size()) + ".");
    }
    if (jointVelVals.size() != jointCount) {
      throw std::runtime_error("joint_vel size mismatch at line " + std::to_string(lineNumber) + ": expected " +
                               std::to_string(jointCount) + ", got " + std::to_string(jointVelVals.size()) + ".");
    }
    if (bodyPosVals.size() != 3 * bodyCount) {
      throw std::runtime_error("body_pos size mismatch at line " + std::to_string(lineNumber) + ": expected " +
                               std::to_string(3 * bodyCount) + ", got " + std::to_string(bodyPosVals.size()) + ".");
    }
    if (bodyQuatVals.size() != 4 * bodyCount) {
      throw std::runtime_error("body_quat size mismatch at line " + std::to_string(lineNumber) + ": expected " +
                               std::to_string(4 * bodyCount) + ", got " + std::to_string(bodyQuatVals.size()) + ".");
    }

    LocalReferenceFrame frame;
    frame.jointPosition = Eigen::Map<const vector_t>(jointPosVals.data(), jointPosVals.size());
    frame.jointVelocity = Eigen::Map<const vector_t>(jointVelVals.data(), jointVelVals.size());
    frame.bodyPositions.reserve(bodyCount);
    frame.bodyOrientations.reserve(bodyCount);

    for (size_t i = 0; i < bodyCount; ++i) {
      vector3_t pos;
      pos << bodyPosVals[3 * i], bodyPosVals[3 * i + 1], bodyPosVals[3 * i + 2];
      frame.bodyPositions.push_back(pos);

      quaternion_t ori(bodyQuatVals[4 * i], bodyQuatVals[4 * i + 1], bodyQuatVals[4 * i + 2], bodyQuatVals[4 * i + 3]);
      if (ori.norm() <= 1e-9) {
        throw std::runtime_error("body_quat has near-zero norm at line " + std::to_string(lineNumber) + ", body index " +
                                 std::to_string(i) + ".");
      }
      ori.normalize();
      frame.bodyOrientations.push_back(ori);
    }

    localReferenceFrames_.push_back(std::move(frame));
  }

  if (localReferenceFrames_.empty()) {
    throw std::runtime_error("Local reference file is empty: " + localReferencePath_);
  }

  std::cout << '\t' << "Loaded local reference frames: " << localReferenceFrames_.size() << " from " << localReferencePath_
            << '\n';
}
}  // namespace legged
