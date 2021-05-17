// Copyright (c) 2017 Andreas Geiger, Philip Lenz, Raquel Urtasun
// Copyright (c) 2020-2021 Arm Limited
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

// This is a modified version of the Kitti "object development kit"
// available from http://www.cvlibs.net/datasets/kitti/eval_object.php


#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/adapted/c_array.hpp>
#include <common/filesystem.hpp>
#include <common/types.hpp>

#include <cstdio>
#include <cmath>
#include <cstring>
#include <cassert>

#include <iostream>
#include <algorithm>
#include <vector>
#include <numeric>
#include <string>
#include <functional>

#include "kittiobjevalmodule.hpp"

#ifdef BOOST_GEOMETRY_REGISTER_C_ARRAY_CS
BOOST_GEOMETRY_REGISTER_C_ARRAY_CS(cs::cartesian)
#endif

using autoware::common::types::bool8_t;
using autoware::common::types::char8_t;
using autoware::common::types::float64_t;
using std::size_t;

typedef boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<float64_t>> Polygon;

/*=======================================================================
STATIC EVALUATION PARAMETERS
=======================================================================*/

// easy, moderate and hard evaluation level
enum class DIFFICULTY {EASY = 0, MODERATE = 1, HARD = 2};

// evaluation metrics: image, ground or 3D
enum class METRIC {IMAGE = 0, GROUND = 1, BOX3D = 2};

// evaluation parameter
// minimum height for evaluated groundtruth/detections
constexpr int32_t MIN_HEIGHT[3] = {40, 25, 25};
// maximum occlusion level of the groundtruth used for evaluation
constexpr int32_t MAX_OCCLUSION[3] = {0, 1, 2};
// maximum truncation level of the groundtruth used for evaluation
constexpr float64_t MAX_TRUNCATION[3] = {0.15, 0.3, 0.5};

// evaluated object classes
enum class CLASSES {CAR = 0, PEDESTRIAN = 1, CYCLIST = 2};
constexpr int NUM_CLASS = 3;

// parameters varying per class
std::vector<std::basic_string<char8_t>> CLASS_NAMES;
std::vector<std::basic_string<char8_t>> CLASS_NAMES_CAP;
// the minimum overlap required for 2D evaluation on the image/ground plane and 3D evaluation
constexpr float64_t MIN_OVERLAP[3][3] = {{0.7, 0.5, 0.5}, {0.7, 0.5, 0.5}, {0.7, 0.5, 0.5}};

// no. of recall steps that should be evaluated (discretized)
constexpr float64_t N_SAMPLE_PTS = 41;

// initialize class names
void initGlobals()
{
  CLASS_NAMES.push_back("car");
  CLASS_NAMES.push_back("pedestrian");
  CLASS_NAMES.push_back("cyclist");
  CLASS_NAMES_CAP.push_back("Car");
  CLASS_NAMES_CAP.push_back("Pedestrian");
  CLASS_NAMES_CAP.push_back("Cyclist");
}

/*=======================================================================
DATA TYPES FOR EVALUATION
=======================================================================*/

// holding data needed for precision-recall and precision-aos
struct tPrData
{
  std::vector<float64_t> v;           // detection score for computing score thresholds
  float64_t similarity;          // orientation similarity
  int32_t tp;                 // true positives
  int32_t fp;                 // false positives
  int32_t fn;                 // false negatives
  tPrData()
  : similarity(0), tp(0), fp(0), fn(0) {}
};

// holding bounding boxes for ground truth and detections
struct tBox
{
  std::basic_string<char8_t> type;      // object type as car, pedestrian or cyclist,...
  float64_t x1;        // left corner
  float64_t y1;        // top corner
  float64_t x2;        // right corner
  float64_t y2;        // bottom corner
  float64_t alpha;     // image orientation
  tBox(
    std::basic_string<char8_t> type, float64_t x1, float64_t y1, float64_t x2, float64_t y2,
    float64_t alpha)
  : type(type), x1(x1), y1(y1), x2(x2), y2(y2), alpha(alpha) {}
};

// holding ground truth data
struct tGroundtruth
{
  tBox box;           // object type, box, orientation
  float64_t truncation;  // truncation 0..1
  int32_t occlusion;  // occlusion 0,1,2 (non, partly, fully)
  float64_t ry;
  float64_t t1, t2, t3;
  float64_t h, w, l;
  tGroundtruth()
  : box(tBox("invalild", -1, -1, -1, -1, -10)), truncation(-1), occlusion(-1) {}
  tGroundtruth(tBox box, float64_t truncation, int32_t occlusion)
  : box(box), truncation(truncation), occlusion(occlusion) {}
  tGroundtruth(
    std::basic_string<char8_t> type, float64_t x1, float64_t y1, float64_t x2,
    float64_t y2, float64_t alpha, float64_t truncation, int32_t occlusion)
  : box(tBox(type, x1, y1, x2, y2, alpha)), truncation(truncation), occlusion(occlusion) {}
};

// holding detection data
struct tDetection
{
  tBox box;       // object type, box, orientation
  float64_t thresh;  // detection score
  float64_t ry;
  float64_t t1, t2, t3;
  float64_t h, w, l;
  tDetection()
  : box(tBox("invalid", -1, -1, -1, -1, -10)), thresh(-1000) {}
  tDetection(tBox box, float64_t thresh)
  : box(box), thresh(thresh) {}
  tDetection(
    std::basic_string<char8_t> type, float64_t x1, float64_t y1, float64_t x2,
    float64_t y2, float64_t alpha, float64_t thresh)
  : box(tBox(type, x1, y1, x2, y2, alpha)), thresh(thresh) {}
};


/*=======================================================================
FUNCTIONS TO LOAD DETECTION AND GROUND TRUTH DATA ONCE, SAVE RESULTS
=======================================================================*/
std::vector<tDetection> loadDetections(
  std::basic_string<char8_t> file_name, bool8_t & compute_aos,
  std::vector<bool8_t> & eval_image, std::vector<bool8_t> & eval_ground,
  std::vector<bool8_t> & eval_3d, bool8_t & success)
{
  // holds all detections (ignored detections are indicated by an index vector
  std::vector<tDetection> detections;
  FILE * fp = fopen(file_name.c_str(), "r");
  if (!fp) {
    success = false;
    return detections;
  }
  while (!feof(fp)) {
    tDetection d;
    float64_t trash;
    char8_t str[255];
    if (fscanf(
        fp, "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
        str, &trash, &trash, &d.box.alpha, &d.box.x1, &d.box.y1,
        &d.box.x2, &d.box.y2, &d.h, &d.w, &d.l, &d.t1, &d.t2, &d.t3,
        &d.ry, &d.thresh) == 16)
    {
      d.box.type = str;
      detections.push_back(d);

      // orientation=-10 is invalid, AOS is not evaluated if at least one orientation is invalid
      if (d.box.alpha == -10) {
        compute_aos = false;
      }

      // a class is only evaluated if it is detected at least once
      for (size_t c = 0; c < NUM_CLASS; c++) {
        if (!strcasecmp(
            d.box.type.c_str(),
            CLASS_NAMES[c].c_str()) || !strcasecmp(d.box.type.c_str(), CLASS_NAMES_CAP[c].c_str()))
        {
          if (!eval_image[c] && d.box.x1 >= 0) {
            eval_image[c] = true;
          }
          if (!eval_ground[c] && d.t1 != -1000 && d.t3 != -1000 && d.w > 0 && d.l > 0) {
            eval_ground[c] = true;
          }
          if (!eval_3d[c] && d.t1 != -1000 && d.t2 != -1000 && d.t3 != -1000 && d.h > 0 &&
            d.w > 0 && d.l > 0)
          {
            eval_3d[c] = true;
          }
          break;
        }
      }
    }
  }

  fclose(fp);
  success = true;
  return detections;
}

std::vector<tGroundtruth> loadGroundtruth(std::basic_string<char8_t> file_name, bool8_t & success)
{
  // holds all ground truth (ignored ground truth is indicated by an index vector
  std::vector<tGroundtruth> groundtruth;
  FILE * fp = fopen(file_name.c_str(), "r");
  if (!fp) {
    success = false;
    return groundtruth;
  }
  while (!feof(fp)) {
    tGroundtruth g;
    char8_t str[255];
    if (fscanf(
        fp, "%s %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
        str, &g.truncation, &g.occlusion, &g.box.alpha,
        &g.box.x1, &g.box.y1, &g.box.x2, &g.box.y2,
        &g.h, &g.w, &g.l, &g.t1,
        &g.t2, &g.t3, &g.ry) == 15)
    {
      g.box.type = str;
      groundtruth.push_back(g);
    }
  }
  fclose(fp);
  success = true;
  return groundtruth;
}

void saveStats(
  const std::vector<float64_t> & precision, const std::vector<float64_t> & aos, FILE * fp_det,
  FILE * fp_ori)
{
  // save precision to file
  if (precision.empty()) {
    return;
  }
  for (uint32_t i = 0; i < precision.size(); i++) {
    fprintf(fp_det, "%f ", precision[i]);
  }
  fprintf(fp_det, "\n");

  // save orientation similarity, only if there were no invalid orientation entries in submission
  // (alpha=-10)
  if (aos.empty()) {
    return;
  }
  for (uint32_t i = 0; i < aos.size(); i++) {
    fprintf(fp_ori, "%f ", aos[i]);
  }
  fprintf(fp_ori, "\n");
}

/*=======================================================================
EVALUATION HELPER FUNCTIONS
=======================================================================*/

// criterion defines whether the overlap is computed with respect to both areas (ground truth and
// detection) or with respect to box a or b (detection and "dontcare" areas)
inline float64_t imageBoxOverlap(tBox a, tBox b, int32_t criterion = -1)
{
  // overlap is invalid in the beginning
  float64_t o = -1;

  // get overlapping area
  float64_t x1 = std::max(a.x1, b.x1);
  float64_t y1 = std::max(a.y1, b.y1);
  float64_t x2 = std::min(a.x2, b.x2);
  float64_t y2 = std::min(a.y2, b.y2);

  // compute width and height of overlapping area
  float64_t w = x2 - x1;
  float64_t h = y2 - y1;

  // set invalid entries to 0 overlap
  if (w <= 0 || h <= 0) {
    return 0;
  }

  // get overlapping areas
  float64_t inter = w * h;
  float64_t a_area = (a.x2 - a.x1) * (a.y2 - a.y1);
  float64_t b_area = (b.x2 - b.x1) * (b.y2 - b.y1);

  // intersection over union overlap depending on users choice
  if (criterion == -1) {   // union
    o = inter / (a_area + b_area - inter);
  } else if (criterion == 0) {  // bbox_a
    o = inter / a_area;
  } else if (criterion == 1) {  // bbox_b
    o = inter / b_area;
  }

  // overlap
  return o;
}

inline float64_t imageBoxOverlap(tDetection a, tGroundtruth b, int32_t criterion = -1)
{
  return imageBoxOverlap(a.box, b.box, criterion);
}

// compute polygon of an oriented bounding box
template<typename T>
Polygon toPolygon(const T & g)
{
  boost::numeric::ublas::matrix<float64_t> mref(2, 2);
  mref(0, 0) = cos(g.ry); mref(0, 1) = sin(g.ry);
  mref(1, 0) = -sin(g.ry); mref(1, 1) = cos(g.ry);

  boost::numeric::ublas::matrix<float64_t> corners(2, 4);
  float64_t data[] = {g.l / 2, g.l / 2, -g.l / 2, -g.l / 2,
    g.w / 2, -g.w / 2, -g.w / 2, g.w / 2};
  std::copy(data, data + 8, corners.data().begin());
  boost::numeric::ublas::matrix<float64_t> gc = prod(mref, corners);
  for (size_t i = 0; i < 4; ++i) {
    gc(0, i) += g.t1;
    gc(1, i) += g.t3;
  }

  float64_t points[][2] =
  {{gc(0, 0), gc(1, 0)}, {gc(0, 1), gc(1, 1)}, {gc(0, 2), gc(1, 2)}, {gc(0, 3), gc(1, 3)}, {gc(
        0,
        0),
      gc(1, 0)}};
  Polygon poly;
  boost::geometry::append(poly, points);
  return poly;
}

// measure overlap between bird's eye view bounding boxes, parametrized by (ry, l, w, tx, tz)
inline float64_t groundBoxOverlap(tDetection d, tGroundtruth g, int32_t criterion = -1)
{
  Polygon gp = toPolygon(g);
  Polygon dp = toPolygon(d);

  std::vector<Polygon> in, un;
  boost::geometry::intersection(gp, dp, in);
  boost::geometry::union_(gp, dp, un);

  float64_t inter_area = in.empty() ? 0 : boost::geometry::area(in.front());
  float64_t union_area = boost::geometry::area(un.front());
  float64_t o = 0;
  if (criterion == -1) {  // union
    o = inter_area / union_area;
  } else if (criterion == 0) {  // bbox_a
    o = inter_area / boost::geometry::area(dp);
  } else if (criterion == 1) {  // bbox_b
    o = inter_area / boost::geometry::area(gp);
  }

  return o;
}

// measure overlap between 3D bounding boxes, parametrized by (ry, h, w, l, tx, ty, tz)
inline float64_t box3DOverlap(tDetection d, tGroundtruth g, int32_t criterion = -1)
{
  Polygon gp = toPolygon(g);
  Polygon dp = toPolygon(d);

  std::vector<Polygon> in, un;
  boost::geometry::intersection(gp, dp, in);
  boost::geometry::union_(gp, dp, un);

  float64_t ymax = std::min(d.t2, g.t2);
  float64_t ymin = std::max(d.t2 - d.h, g.t2 - g.h);

  float64_t inter_area = in.empty() ? 0 : boost::geometry::area(in.front());
  float64_t inter_vol = inter_area * std::max(0.0, ymax - ymin);

  float64_t det_vol = d.h * d.l * d.w;
  float64_t gt_vol = g.h * g.l * g.w;

  float64_t o = 0;
  if (criterion == -1) {  // union
    o = inter_vol / (det_vol + gt_vol - inter_vol);
  } else if (criterion == 0) {  // bbox_a
    o = inter_vol / det_vol;
  } else if (criterion == 1) {  // bbox_b
    o = inter_vol / gt_vol;
  }

  return o;
}

std::vector<float64_t> getThresholds(std::vector<float64_t> & v, float64_t n_groundtruth)
{
  // holds scores needed to compute N_SAMPLE_PTS recall values
  std::vector<float64_t> t;

  // sort scores in descending order
  // (highest score is assumed to give best/most confident detections)
  sort(v.begin(), v.end(), std::greater<float64_t>());

  // get scores for linearly spaced recall
  float64_t current_recall = 0;
  for (uint32_t i = 0; i < v.size(); i++) {
    // check if right-hand-side recall with respect to current recall is close than left-hand-side
    // one. In this case, skip the current detection score
    float64_t l_recall, r_recall /*, recall*/;
    l_recall = static_cast<float64_t>(i + 1) / n_groundtruth;
    if (i < (v.size() - 1)) {
      r_recall = static_cast<float64_t>(i + 2) / n_groundtruth;
    } else {
      r_recall = l_recall;
    }

    if ( (r_recall - current_recall) < (current_recall - l_recall) && i < (v.size() - 1)) {
      continue;
    }

    // left recall is the best approximation, so use this and goto next recall step for
    // approximation recall = l_recall;

    // the next recall step was reached
    t.push_back(v[i]);
    current_recall += 1.0 / (N_SAMPLE_PTS - 1.0);
  }
  return t;
}

void cleanData(
  CLASSES current_class, const std::vector<tGroundtruth> & gt,
  const std::vector<tDetection> & det, std::vector<int32_t> & ignored_gt,
  std::vector<tGroundtruth> & dc, std::vector<int32_t> & ignored_det, int32_t & n_gt,
  DIFFICULTY difficulty)
{
  // extract ground truth bounding boxes for current evaluation class
  for (uint32_t i = 0; i < gt.size(); i++) {
    // only bounding boxes with a minimum height are used for evaluation
    float64_t height = gt[i].box.y2 - gt[i].box.y1;

    // neighboring classes are ignored ("van" for "car" and "person_sitting" for "pedestrian")
    // (lower/upper cases are ignored)
    int32_t valid_class;
    const char8_t * class_name = CLASS_NAMES[static_cast<size_t>(current_class)].c_str();
    const char8_t * box_type = gt[i].box.type.c_str();

    // all classes without a neighboring class
    if (!strcasecmp(box_type, class_name)) {
      valid_class = 1;
    } else if (!strcasecmp(class_name, "Pedestrian") && !strcasecmp("Person_sitting", box_type)) {
      // classes with a neighboring class
      valid_class = 0;
    } else if (!strcasecmp(class_name, "Car") && !strcasecmp("Van", box_type)) {
      valid_class = 0;
    } else {
      // classes not used for evaluation
      valid_class = -1;
    }

    // ground truth is ignored, if occlusion, truncation exceeds the difficulty or ground truth is
    // too small (doesn't count as FN nor TP, although detections may be assigned)
    bool8_t ignore = false;
    if (gt[i].occlusion > MAX_OCCLUSION[static_cast<size_t>(difficulty)] ||
      gt[i].truncation > MAX_TRUNCATION[static_cast<size_t>(difficulty)] ||
      height <= MIN_HEIGHT[static_cast<size_t>(difficulty)])
    {
      ignore = true;
    }

    // set ignored vector for ground truth
    // current class and not ignored (total no. of ground truth is detected for recall denominator)
    if (valid_class == 1 && !ignore) {
      ignored_gt.push_back(0);
      n_gt++;
    } else if (valid_class == 0 || (ignore && valid_class == 1)) {
      // neighboring class, or current class but ignored
      ignored_gt.push_back(1);
    } else {
      // all other classes which are FN in the evaluation
      ignored_gt.push_back(-1);
    }
  }

  // extract dontcare areas
  for (uint32_t i = 0; i < gt.size(); i++) {
    if (!strcasecmp("DontCare", gt[i].box.type.c_str())) {
      dc.push_back(gt[i]);
    }
  }

  // extract detections bounding boxes of the current class
  for (uint32_t i = 0; i < det.size(); i++) {
    // neighboring classes are not evaluated
    int32_t valid_class;
    if (!strcasecmp(
        det[i].box.type.c_str(),
        CLASS_NAMES[static_cast<size_t>(current_class)].c_str()))
    {
      valid_class = 1;
    } else {
      valid_class = -1;
    }

    auto height = fabs(det[i].box.y1 - det[i].box.y2);

    // set ignored vector for detections
    if (height < MIN_HEIGHT[static_cast<size_t>(difficulty)]) {
      ignored_det.push_back(1);
    } else if (valid_class == 1) {
      ignored_det.push_back(0);
    } else {
      ignored_det.push_back(-1);
    }
  }
}

tPrData computeStatistics(
  CLASSES current_class, const std::vector<tGroundtruth> & gt,
  const std::vector<tDetection> & det, const std::vector<tGroundtruth> & dc,
  const std::vector<int32_t> & ignored_gt, const std::vector<int32_t> & ignored_det,
  bool8_t compute_fp, float64_t (* boxoverlap)(tDetection, tGroundtruth, int32_t),
  METRIC metric, bool8_t compute_aos = false, float64_t thresh = 0)
{
  tPrData stat = tPrData();
  const float64_t NO_DETECTION = -10000000;
  // holds angular difference for TPs (needed for AOS evaluation)
  std::vector<float64_t> delta;
  // holds wether a detection was assigned to a valid or ignored ground truth
  std::vector<bool8_t> assigned_detection;
  assigned_detection.assign(det.size(), false);
  std::vector<bool8_t> ignored_threshold;
  // holds detections with a threshold lower than thresh if FP are computed
  ignored_threshold.assign(det.size(), false);

  // detections with a low score are ignored for computing precision (needs FP)
  if (compute_fp) {
    for (uint32_t i = 0; i < det.size(); i++) {
      if (det[i].thresh < thresh) {
        ignored_threshold[i] = true;
      }
    }
  }

  // evaluate all ground truth boxes
  for (uint32_t i = 0; i < gt.size(); i++) {
    // this ground truth is not of the current or a neighboring class and therefore ignored
    if (ignored_gt[i] == -1) {
      continue;
    }

    /*=======================================================================
    find candidates (overlap with ground truth > 0.5) (logical len(det))
    =======================================================================*/
    uint32_t det_idx = 0;
    float64_t valid_detection = NO_DETECTION;
    float64_t max_overlap = 0;

    // search for a possible detection
    bool8_t assigned_ignored_det = false;
    for (uint32_t j = 0; j < det.size(); j++) {
      // detections not of the current class, already assigned or with a low threshold are ignored
      if (ignored_det[j] == -1) {
        continue;
      }
      if (assigned_detection[j]) {
        continue;
      }
      if (ignored_threshold[j]) {
        continue;
      }

      // find the maximum score for the candidates and get idx of respective detection
      float64_t overlap = boxoverlap(det[j], gt[i], -1);

      // for computing recall thresholds, the candidate with highest score is considered
      if (!compute_fp &&
        overlap > MIN_OVERLAP[static_cast<size_t>(metric)][static_cast<size_t>(current_class)] &&
        det[j].thresh > valid_detection)
      {
        det_idx = j;
        valid_detection = det[j].thresh;
      } else if (compute_fp &&  // NOLINT
        overlap > MIN_OVERLAP[static_cast<size_t>(metric)][static_cast<size_t>(current_class)] &&
        (overlap > max_overlap || assigned_ignored_det) && ignored_det[j] == 0)
      {
        // for computing pr curve values, the candidate with the greatest overlap is considered
        // if the greatest overlap is an ignored detection (min_height), the overlapping detection
        // is used
        max_overlap = overlap;
        det_idx = j;
        valid_detection = 1;
        assigned_ignored_det = false;
      } else if (compute_fp &&  // NOLINT
        overlap > MIN_OVERLAP[static_cast<size_t>(metric)][static_cast<size_t>(current_class)] &&
        valid_detection == NO_DETECTION && ignored_det[j] == 1)
      {
        det_idx = j;
        valid_detection = 1;
        assigned_ignored_det = true;
      }
    }

    /*=======================================================================
    compute TP, FP and FN
    =======================================================================*/

    // nothing was assigned to this valid ground truth
    if (valid_detection == NO_DETECTION && ignored_gt[i] == 0) {
      stat.fn++;
    } else if (valid_detection != NO_DETECTION &&  // NOLINT
      (ignored_gt[i] == 1 || ignored_det[det_idx] == 1))
    {
      // only evaluate valid ground truth <=> detection assignments (considering difficulty level)
      assigned_detection[det_idx] = true;
    } else if (valid_detection != NO_DETECTION) {
      // found a valid true positive
      // write highest score to threshold vector
      stat.tp++;
      stat.v.push_back(det[det_idx].thresh);

      // compute angular difference of detection and ground truth if valid detection orientation was
      // provided
      if (compute_aos) {
        delta.push_back(gt[i].box.alpha - det[det_idx].box.alpha);
      }

      // clean up
      assigned_detection[det_idx] = true;
    }
  }

  // if FP are requested, consider stuff area
  if (compute_fp) {
    // count fp
    for (uint32_t i = 0; i < det.size(); i++) {
      // count false positives if required (height smaller than required is ignored (ignored_det==1)
      if (!(assigned_detection[i] || ignored_det[i] == -1 || ignored_det[i] == 1 ||
        ignored_threshold[i]))
      {
        stat.fp++;
      }
    }

    // do not consider detections overlapping with stuff area
    int32_t nstuff = 0;
    for (uint32_t i = 0; i < dc.size(); i++) {
      for (uint32_t j = 0; j < det.size(); j++) {
        // detections not of the current class, already assigned, with a low threshold or a low
        // minimum height are ignored
        if (assigned_detection[j]) {
          continue;
        }
        if (ignored_det[j] == -1 || ignored_det[j] == 1) {
          continue;
        }
        if (ignored_threshold[j]) {
          continue;
        }

        // compute overlap and assign to stuff area, if overlap exceeds class specific value
        float64_t overlap = boxoverlap(det[j], dc[i], 0);
        auto metric_s = static_cast<size_t>(metric);
        auto current_class_s = static_cast<size_t>(current_class);
        if (overlap > MIN_OVERLAP[metric_s][current_class_s]) {
          assigned_detection[j] = true;
          nstuff++;
        }
      }
    }

    // FP = no. of all not to ground truth assigned detections - detections assigned to stuff areas
    stat.fp -= nstuff;

    // if all orientation values are valid, the AOS is computed
    if (compute_aos) {
      std::vector<float64_t> tmp;

      // FP have a similarity of 0, for all TP compute AOS
      tmp.assign(static_cast<size_t>(stat.fp), 0);
      for (uint32_t i = 0; i < delta.size(); i++) {
        tmp.push_back((1.0 + cos(delta[i])) / 2.0);
      }

      // be sure, that all orientation deltas are computed
      assert(tmp.size() == static_cast<size_t>(stat.fp + stat.tp));
      assert(delta.size() == static_cast<size_t>(stat.tp));

      // get the mean orientation similarity for this image
      if (stat.tp > 0 || stat.fp > 0) {
        stat.similarity = accumulate(tmp.begin(), tmp.end(), 0.0);
      } else {
        // there was neither a FP nor a TP, so the similarity is ignored in the evaluation
        stat.similarity = -1;
      }
    }
  }
  return stat;
}

/*=======================================================================
EVALUATE CLASS-WISE
=======================================================================*/

bool8_t eval_class(
  FILE * fp_det, FILE * fp_ori, CLASSES current_class,
  const std::vector<std::vector<tGroundtruth>> & groundtruth,
  const std::vector<std::vector<tDetection>> & detections, bool8_t compute_aos,
  float64_t (* boxoverlap)(tDetection, tGroundtruth, int32_t),
  std::vector<float64_t> & precision, std::vector<float64_t> & aos,
  DIFFICULTY difficulty, METRIC metric)
{
  assert(groundtruth.size() == detections.size());

  // init
  // total no. of gt (denominator of recall)
  int32_t n_gt = 0;
  // detection scores, evaluated for recall discretization
  std::vector<float64_t> v, thresholds;
  // index of ignored gt detection for current class/difficulty
  std::vector<std::vector<int32_t>> ignored_gt, ignored_det;
  // index of dontcare areas, included in ground truth
  std::vector<std::vector<tGroundtruth>> dontcare;

  // for all test images do
  for (uint32_t i = 0; i < groundtruth.size(); i++) {
    // holds ignored ground truth, ignored detections and dontcare areas for current frame
    std::vector<int32_t> i_gt, i_det;
    std::vector<tGroundtruth> dc;

    // only evaluate objects of current class and ignore occluded, truncated objects
    cleanData(current_class, groundtruth[i], detections[i], i_gt, dc, i_det, n_gt, difficulty);
    ignored_gt.push_back(i_gt);
    ignored_det.push_back(i_det);
    dontcare.push_back(dc);

    // compute statistics to get recall values
    tPrData pr_tmp = tPrData();
    pr_tmp = computeStatistics(
      current_class, groundtruth[i], detections[i], dc, i_gt, i_det, false,
      boxoverlap, metric);

    // add detection scores to vector over all images
    for (uint32_t j = 0; j < pr_tmp.v.size(); j++) {
      v.push_back(pr_tmp.v[j]);
    }
  }

  // get scores that must be evaluated for recall discretization
  thresholds = getThresholds(v, n_gt);

  // compute TP,FP,FN for relevant scores
  std::vector<tPrData> pr;
  pr.assign(thresholds.size(), tPrData());
  for (uint32_t i = 0; i < groundtruth.size(); i++) {
    // for all scores/recall thresholds do:
    for (uint32_t t = 0; t < thresholds.size(); t++) {
      tPrData tmp = tPrData();
      tmp = computeStatistics(
        current_class, groundtruth[i], detections[i], dontcare[i],
        ignored_gt[i], ignored_det[i], true, boxoverlap, metric,
        compute_aos, thresholds[t]);

      // add no. of TP, FP, FN, AOS for current frame to total evaluation for current threshold
      pr[t].tp += tmp.tp;
      pr[t].fp += tmp.fp;
      pr[t].fn += tmp.fn;
      if (tmp.similarity != -1) {
        pr[t].similarity += tmp.similarity;
      }
    }
  }

  // compute recall, precision and AOS
  std::vector<float64_t> recall;
  precision.assign(N_SAMPLE_PTS, 0);
  if (compute_aos) {
    aos.assign(N_SAMPLE_PTS, 0);
  }
  float64_t r = 0;
  for (uint32_t i = 0; i < thresholds.size(); i++) {
    r = pr[i].tp / static_cast<float64_t>(pr[i].tp + pr[i].fn);
    recall.push_back(r);
    precision[i] = pr[i].tp / static_cast<float64_t>(pr[i].tp + pr[i].fp);
    if (compute_aos) {
      aos[i] = pr[i].similarity / static_cast<float64_t>(pr[i].tp + pr[i].fp);
    }
  }

  // filter precision and AOS using max_{i..end}(precision)
  for (uint32_t i = 0; i < thresholds.size(); i++) {
    precision[i] = *max_element(precision.begin() + i, precision.end());
    if (compute_aos) {
      aos[i] = *max_element(aos.begin() + i, aos.end());
    }
  }

  // save statisics and finish with success
  saveStats(precision, aos, fp_det, fp_ori);
  return true;
}

void saveAndPlotPlots(
  std::basic_string<char8_t> dir_name, std::basic_string<char8_t> file_name,
  std::basic_string<char8_t> obj_type, std::vector<float64_t> vals[], bool8_t is_aos)
{
  // save plot data to file
  FILE * fp = fopen((dir_name + "/" + file_name + ".txt").c_str(), "w");
  printf("save %s\n", (dir_name + "/" + file_name + ".txt").c_str());
  for (size_t i = 0; i < static_cast<size_t>(N_SAMPLE_PTS); i++) {
    fprintf(
      fp, "%f %f %f %f\n", static_cast<float64_t>(i) / (N_SAMPLE_PTS - 1.0), vals[0][i], vals[1][i],
      vals[2][i]);
  }
  fclose(fp);

  // create png + eps
  for (int32_t j = 0; j < 2; j++) {
    std::basic_string<char8_t> type = "_png";

    if (j != 0) {
      type = "_pdf";
    }
    // open file
    FILE * fp = fopen((dir_name + "/" + file_name + type + ".gp").c_str(), "w");

    // save gnuplot instructions
    if (j == 0) {
      fprintf(fp, "set term png size 450,315 font \"Helvetica\" 11\n");
      fprintf(fp, "set output \"%s.png\"\n", file_name.c_str());
    } else {
      fprintf(fp, "set term postscript eps enhanced color font \"Helvetica\" 20\n");
      fprintf(fp, "set output \"%s.eps\"\n", file_name.c_str());
    }

    // set labels and ranges
    fprintf(fp, "set size ratio 0.7\n");
    fprintf(fp, "set xrange [0:1]\n");
    fprintf(fp, "set yrange [0:1]\n");
    fprintf(fp, "set xlabel \"Recall\"\n");
    if (!is_aos) {fprintf(fp, "set ylabel \"Precision\"\n");} else {
      fprintf(fp, "set ylabel \"Orientation Similarity\"\n");
    }
    obj_type[0] = static_cast<char8_t>(toupper(obj_type[0]));
    fprintf(fp, "set title \"%s\"\n", obj_type.c_str());

    // line width
    int32_t lw = 5;
    if (j == 0) {lw = 3;}

    // plot error curve
    fprintf(fp, "plot ");
    fprintf(fp, "\"%s.txt\" using 1:2 title 'Easy' with lines ls 1 lw %d,", file_name.c_str(), lw);
    fprintf(
      fp, "\"%s.txt\" using 1:3 title 'Moderate' with lines ls 2 lw %d,",
      file_name.c_str(), lw);
    fprintf(fp, "\"%s.txt\" using 1:4 title 'Hard' with lines ls 3 lw %d", file_name.c_str(), lw);

    // close file
    fclose(fp);
  }
}

namespace kittisdk
{

extern "C"
int32_t eval(
  std::basic_string<char8_t> ground_truth_path, std::basic_string<char8_t> detection_path,
  std::basic_string<char8_t> output_path, bool8_t eval_2d_res, bool8_t eval_ground_red,
  bool8_t eval_3d_res, bool8_t print_stdout, bool8_t create_plot, int32_t n_testimages)
{
  // set some global parameters
  initGlobals();

  // ground truth and result directories
  std::basic_string<char8_t> result_dir = output_path;
  std::basic_string<char8_t> plot_dir = result_dir + "/plot";

  // Check folder existance
  if (!ghc::filesystem::is_directory(ghc::filesystem::path(ground_truth_path)) ||
    !ghc::filesystem::is_directory(ghc::filesystem::path(detection_path)) ||
    !ghc::filesystem::is_directory(ghc::filesystem::path(result_dir)) )
  {
    if (print_stdout) {
      printf("Folder structure error, please check the existance of: \n");
      printf("%s\n", ground_truth_path.c_str());
      printf("%s\n", detection_path.c_str());
      printf("%s\n", result_dir.c_str());
    }
    return 0;
  }

  if (create_plot && !ghc::filesystem::is_directory(ghc::filesystem::path(plot_dir))) {
    printf("Folder structure error, please check the existance of: \n");
    printf("%s\n", plot_dir.c_str());
  }

  // hold detections and ground truth in memory
  std::vector<std::vector<tGroundtruth>> groundtruth;
  std::vector<std::vector<tDetection>> detections;

  // holds wether orientation similarity shall be computed (might be set to false while loading
  // detections) and which labels where provided by this submission
  bool8_t compute_aos = true;
  std::vector<bool8_t> eval_image(NUM_CLASS, false);
  std::vector<bool8_t> eval_ground(NUM_CLASS, false);
  std::vector<bool8_t> eval_3d(NUM_CLASS, false);

  // for all images read groundtruth and detections
  if (print_stdout) {
    printf("Loading detections...\n");
  }

  for (int32_t i = 0; i < n_testimages; i++) {
    // file name
    char8_t file_name[256];
    snprintf(file_name, sizeof(file_name), "%06d.txt", i);

    // read ground truth and result poses
    bool8_t gt_success, det_success;
    std::vector<tGroundtruth> gt = loadGroundtruth(ground_truth_path + "/" + file_name, gt_success);
    std::vector<tDetection> det = loadDetections(
      detection_path + "/" + file_name,
      compute_aos, eval_image, eval_ground, eval_3d, det_success);
    groundtruth.push_back(gt);
    detections.push_back(det);

    // check for errors
    if (!gt_success) {
      if (print_stdout) {
        printf("ERROR: Couldn't read: %s of ground truth.\n", file_name);
      }
      return 0;
    }
    if (!det_success) {
      if (print_stdout) {
        printf("ERROR: Couldn't read: %s\n", file_name);
      }
      return 0;
    }
  }
  if (print_stdout) {
    printf("  done.\n");
  }

  // holds pointers for result files
  FILE * fp_det = 0, * fp_ori = 0;

  if (eval_2d_res) {
    // eval image 2D bounding boxes
    for (size_t c = 0; c < static_cast<size_t>(NUM_CLASS); c++) {
      CLASSES cls = (CLASSES)c;

      if (eval_image[c]) {
        if (print_stdout) {
          printf("Starting 2D evaluation (%s) ...\n", CLASS_NAMES[c].c_str());
        }
        fp_det = fopen((result_dir + "/stats_" + CLASS_NAMES[c] + "_detection.txt").c_str(), "w");
        if (compute_aos) {
          fp_ori =
            fopen((result_dir + "/stats_" + CLASS_NAMES[c] + "_orientation.txt").c_str(), "w");
        }
        std::vector<float64_t> precision[3], aos[3];
        if (!eval_class(
            fp_det, fp_ori, cls, groundtruth, detections, compute_aos, imageBoxOverlap,
            precision[0], aos[0], DIFFICULTY::EASY, METRIC::IMAGE) ||
          !eval_class(
            fp_det, fp_ori, cls, groundtruth, detections, compute_aos, imageBoxOverlap,
            precision[1], aos[1], DIFFICULTY::MODERATE, METRIC::IMAGE) ||
          !eval_class(
            fp_det, fp_ori, cls, groundtruth, detections, compute_aos, imageBoxOverlap,
            precision[2], aos[2], DIFFICULTY::HARD, METRIC::IMAGE))
        {
          if (print_stdout) {
            printf("%s evaluation failed.\n", CLASS_NAMES[c].c_str());
          }
          return 0;
        }
        fclose(fp_det);
        if (create_plot) {
          saveAndPlotPlots(plot_dir, CLASS_NAMES[c] + "_detection", CLASS_NAMES[c], precision, 0);
        }
        if (compute_aos) {
          if (create_plot) {
            saveAndPlotPlots(plot_dir, CLASS_NAMES[c] + "_orientation", CLASS_NAMES[c], aos, 1);
          }
          fclose(fp_ori);
        }
        if (print_stdout) {
          printf("  done.\n");
        }
      }
    }
  }

  // don't evaluate AOS for birdview boxes and 3D boxes
  compute_aos = false;

  if (eval_ground_red) {
    // eval bird's eye view bounding boxes
    for (size_t c = 0; c < static_cast<size_t>(NUM_CLASS); c++) {
      CLASSES cls = (CLASSES)c;

      if (eval_ground[c]) {
        if (print_stdout) {
          printf("Starting bird's eye evaluation (%s) ...\n", CLASS_NAMES[c].c_str());
        }
        fp_det = fopen(
          (result_dir + "/stats_" + CLASS_NAMES[c] + "_detection_ground.txt").c_str(), "w");
        std::vector<float64_t> precision[3], aos[3];
        if (!eval_class(
            fp_det, fp_ori, cls, groundtruth, detections, compute_aos, groundBoxOverlap,
            precision[0], aos[0], DIFFICULTY::EASY, METRIC::GROUND) ||
          !eval_class(
            fp_det, fp_ori, cls, groundtruth, detections, compute_aos, groundBoxOverlap,
            precision[1], aos[1], DIFFICULTY::MODERATE, METRIC::GROUND) ||
          !eval_class(
            fp_det, fp_ori, cls, groundtruth, detections, compute_aos, groundBoxOverlap,
            precision[2], aos[2], DIFFICULTY::HARD, METRIC::GROUND))
        {
          if (print_stdout) {
            printf("%s evaluation failed.\n", CLASS_NAMES[c].c_str());
          }
          return 0;
        }
        fclose(fp_det);
        if (create_plot) {
          saveAndPlotPlots(
            plot_dir, CLASS_NAMES[c] + "_detection_ground", CLASS_NAMES[c],
            precision, 0);
        }
        if (print_stdout) {
          printf("  done.\n");
        }
      }
    }
  }

  if (eval_3d_res) {
    // eval 3D bounding boxes
    for (size_t c = 0; c < static_cast<size_t>(NUM_CLASS); c++) {
      CLASSES cls = (CLASSES)c;

      if (eval_3d[c]) {
        if (print_stdout) {
          printf("Starting 3D evaluation (%s) ...\n", CLASS_NAMES[c].c_str());
        }
        fp_det =
          fopen((result_dir + "/stats_" + CLASS_NAMES[c] + "_detection_3d.txt").c_str(), "w");
        std::vector<float64_t> precision[3], aos[3];
        if (!eval_class(
            fp_det, fp_ori, cls, groundtruth, detections, compute_aos, box3DOverlap,
            precision[0], aos[0], DIFFICULTY::EASY, METRIC::BOX3D) ||
          !eval_class(
            fp_det, fp_ori, cls, groundtruth, detections, compute_aos, box3DOverlap,
            precision[1], aos[1], DIFFICULTY::MODERATE, METRIC::BOX3D) ||
          !eval_class(
            fp_det, fp_ori, cls, groundtruth, detections, compute_aos, box3DOverlap,
            precision[2], aos[2], DIFFICULTY::HARD, METRIC::BOX3D))
        {
          if (print_stdout) {
            printf("%s evaluation failed.\n", CLASS_NAMES[c].c_str());
          }
          return 0;
        }
        fclose(fp_det);
        if (create_plot) {
          saveAndPlotPlots(
            plot_dir, CLASS_NAMES[c] + "_detection_3d", CLASS_NAMES[c], precision,
            0);
        }
        if (print_stdout) {
          printf("  done.\n");
        }
      }
    }
  }

  // success
  return 1;
}

}  // namespace kittisdk
