#pragma once
#include <assert.h>

#include <algorithm>
#include <iostream>
#include <vector>

namespace smartlabel {
class IntervalCount {
 public:
  size_t tp;
  size_t fp;
  size_t fn;

 public:
  IntervalCount() : tp(0), fp(0), fn(0) {}
  IntervalCount(const size_t tp, const size_t fp, const size_t fn)
      : tp(tp), fp(fp), fn(fn) {}
  ~IntervalCount() = default;

  IntervalCount& operator+=(const IntervalCount& other) {
    tp += other.tp;
    fp += other.fp;
    fn += other.fn;
    return *this;
  }

  friend std::ostream& operator<<(std::ostream& os,
                                  const IntervalCount& count) {
    os << "tp: " << count.tp << " fp: " << count.fp << " fn: " << count.fn;
    return os;
  }

  // tp
  size_t true_positive() const { return tp; }
  // fp
  size_t false_positive() const { return fp; }
  // fn
  size_t false_negative() const { return fn; }

  // precision
  double precision() const {
    if (tp + fp == 0) {
      return 0;
    }
    return static_cast<double>(tp) / (tp + fp);
  }

  // recall
  double recall() const {
    if (tp + fn == 0) {
      return 0;
    }
    return static_cast<double>(tp) / (tp + fn);
  }

  // f1 score
  double f1_score() const {
    if (precision() + recall() == 0) {
      return 0;
    }
    return 2 * precision() * recall() / (precision() + recall());
  }

  // iou
  double iou() const {
    if (tp + fp + fn == 0) {
      return 0;
    }
    return static_cast<double>(tp) / (tp + fp + fn);
  }
};

class SampleCount {
 public:
  std::vector<IntervalCount> sample;

 public:
  SampleCount() : sample(256, IntervalCount()) {}
  ~SampleCount() = default;
  SampleCount(const size_t size) : sample(size, IntervalCount()) {}

  size_t size() const { return sample.size(); }

  void resize(size_t size) { sample.resize(size, IntervalCount()); }

  SampleCount& operator+=(const SampleCount& other) {
    assert(this->size() == other.size());
    for (size_t i = 0; i < sample.size(); ++i) {
      sample[i] += other.sample[i];
    }
    return *this;
  }

  // Use this function to get the count of a specific interval
  IntervalCount& operator[](const size_t index) {
    assert(index < this->size());
    return sample[index];
  }

  // Use this function to get the total count of all intervals
  IntervalCount total() const {
    IntervalCount total_count;
    for (const auto& interval : sample) {
      total_count += interval;
    }
    return total_count;
  }

  friend std::ostream& operator<<(std::ostream& os, const SampleCount& object) {
    for (size_t i = 0; i < object.size(); ++i) {
      os << "Interval: " << i << " " << object.sample[i] << std::endl;
    }
    return os;
  }
};

class SampleMap {
 public:
  size_t m_interval_size_;
  std::vector<SampleCount> dataset;

 public:
  SampleMap() : m_interval_size_(0), dataset(0) {}
  SampleMap(const size_t sample_size, const size_t interval_size)
      : m_interval_size_(interval_size),
        dataset(sample_size, SampleCount(interval_size)) {}
  ~SampleMap() = default;

  void push_back(const SampleCount& sample) {
    if (dataset.size() == 0) {
      this->m_interval_size_ = sample.size();
    } else {
      assert(interval_size() == sample.size());
    }
    dataset.push_back(sample);
  }

  size_t size() const { return dataset.size(); }
  size_t interval_size() const { return m_interval_size_; }

  friend std::ostream& operator<<(std::ostream& os, const SampleMap& object) {
    for (size_t i = 0; i < object.size(); ++i) {
      os << "Sample: " << i << " " << object.dataset[i] << std::endl;
    }
    return os;
  }

  // Use this function to get the count of a specific sample
  SampleCount& operator[](const size_t index) {
    assert(index < this->size());
    return dataset[index];
  }

  // Use this function to get the total count of all samples
  SampleCount total() const {
    SampleCount total_count(this->interval_size());
    for (const auto& sample : dataset) {
      total_count += sample;
    }
    return total_count;
  }

  // Use this function to get sevreral samples
  SampleCount get_samples(const std::vector<size_t>& indices) const {
    // remove duplicate indices
    auto unique_indices = indices;
    // remove duplicate indices
    std::sort(unique_indices.begin(), unique_indices.end());
    unique_indices.erase(
        std::unique(unique_indices.begin(), unique_indices.end()),
        unique_indices.end());

    SampleCount sample_count(this->interval_size());
    for (const auto& index : unique_indices) {
      assert(index < this->size());
      sample_count += dataset[index];
    }
    return sample_count;
  }

  SampleCount get_samples(const std::vector<bool>& mask) const {
    assert(mask.size() == this->size());
    SampleCount sample_count(this->interval_size());
    for (size_t i = 0; i < mask.size(); ++i) {
      if (mask[i]) {
        sample_count += dataset[i];
      }
    }
    return sample_count;
  }

  SampleCount get_samples(const size_t start, const size_t end) const {
    assert(start < this->size());
    assert(end < this->size());
    assert(start <= end);
    SampleCount sample_count(this->interval_size());
    for (size_t i = start; i <= end; ++i) {
      sample_count += dataset[i];
    }
    return sample_count;
  }
};

}  // namespace smartlabel
