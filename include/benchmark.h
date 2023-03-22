#include <iostream>
#include <vector>

class PointCount {
 public:
  size_t tp;
  size_t fp;
  size_t fn;

  PointCount() : tp(0), fp(0), fn(0) {}
  PointCount(const size_t tp, const size_t fp, const size_t fn)
      : tp(tp), fp(fp), fn(fn) {}
  PointCount operator+=(const PointCount& other) {
    tp += other.tp;
    fp += other.fp;
    fn += other.fn;
    return *this;
  }
  friend std::ostream& operator<<(std::ostream& os, const PointCount& count) {
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

class PointCountMap {
 public:
  std::vector<PointCount> counts;
  PointCountMap() : counts(256, PointCount()) {}
  PointCountMap(const size_t size) : counts(size, PointCount()) {}
  PointCountMap operator+=(const PointCountMap& other) {
    for (size_t i = 0; i < counts.size(); ++i) {
      counts[i] += other.counts[i];
    }
    return *this;
  }
  // overload operator<<
  friend std::ostream& operator<<(std::ostream& os,
                                  const PointCountMap& count_map) {
    for (size_t i = 0; i < count_map.counts.size(); ++i) {
      os << "Interval: " << i << " " << count_map.counts[i] << std::endl;
    }
    return os;
  }
  // overload operator[]
  PointCount& operator[](const size_t index) { return counts[index]; }

  // overload operator+=
  PointCountMap& operator+=(const PointCount& count) {
    for (size_t i = 0; i < counts.size(); ++i) {
      counts[i] += count;
    }
    return *this;
  }

  // total
  PointCount total() const {
    PointCount total_count;
    for (const auto& count : counts) {
      total_count += count;
    }
    return total_count;
  }
};