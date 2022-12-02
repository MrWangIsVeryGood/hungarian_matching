#include <cmath>
#include <iostream>
#include "hm_optimizer.h"
using namespace std;
#define MATCHING_THRESHOLD 100;
struct MyObject {
    int id;
    float x;
    float y;
    float w;
    float h;
    int type;
    int is_matched;
};

// 计算关联代价，目前只用了中心点的欧氏距离，你可以把它改成马氏距离或IOU
float getMatchingCost(MyObject trk_obj, MyObject det_obj) {
    float delta_x = trk_obj.x - det_obj.x;
    float delta_y = trk_obj.y - det_obj.y;
    float matching_cost = sqrt(delta_x * delta_x + delta_y * delta_y);
    //对于不同类型的目标，我们可以设置一个很大的代价，这样它们就不会被匹配
    //你可以在这里加入更多的tricks
    if (trk_obj.type != det_obj.type) {
      matching_cost = 100000;
    }
    return matching_cost;
}
// 计算关联代价矩阵
vector<vector<float>> getCostMatrix(std::vector<MyObject> trk_list, std::vector<MyObject> det_list) {
    vector<vector<float>> cost_matrix;
    for (const auto & i : trk_list) {
      vector<float> cost_matrix_row;
      for (const auto & j : det_list) {
        float cost_matrix_element = getMatchingCost(i, j);
        cost_matrix_row.push_back(cost_matrix_element);
      }
      cost_matrix.push_back(cost_matrix_row);
    }
    return cost_matrix;
}

void matchObjects(std::vector<MyObject>& trk_list, std::vector<MyObject>& det_list) {
  for (auto& i : trk_list) {
    i.is_matched = 0;
  }
  for (auto& i : det_list) {
    i.is_matched = 0;
  }
  int matrix_rows = trk_list.size();
  int matrix_cols = det_list.size();
  vector<vector<float>> cost_matrix = getCostMatrix(trk_list, det_list);
  vector<vector<int>> assign_matrix;
  //这里调用Apollo的匈牙利算法，也就是hm_optimizer.h中的HungarianOptimizer
  //使用起来很简单，只需要把关联代价赋值进去就行了
  auto* optimizer_ = new HungarianOptimizer<float>();
  std::vector<std::pair<size_t, size_t>> assignments;
  optimizer_->costs()->Reserve(1000, 1000);
  optimizer_->costs()->Resize(matrix_rows, matrix_cols);
  for (int i = 0; i < matrix_rows; ++i) {
    for (int j = 0; j < matrix_cols; ++j) {
      (*optimizer_->costs())(i, j) = cost_matrix[i][j];
    }
  }
  //这里是Minimize模式，也就是最小化总体关联代价。如果你想最大化总体关联代价，可以改成Maximize
  optimizer_->Minimize(&assignments);
  std::cout << "assignments: " << assignments.size() << std::endl;
  for (auto& assignment : assignments) {
    int trk_id = assignment.first;
    int det_id = assignment.second;
    float threshold = MATCHING_THRESHOLD;
    bool if_cost_under_threshold = (cost_matrix[trk_id][det_id] < threshold);
    bool if_type_matched = (trk_list[trk_id].type == det_list[det_id].type);
    //这里可以判定是否是有效关联
    if (if_cost_under_threshold && if_type_matched) {
      trk_list[trk_id].is_matched = 1;
      det_list[det_id].is_matched = 1;
      //此时关联成功了，可以做接下来的操作，比如更新跟踪器的状态
    }
  }
}

int main() {
    std::cout << "Hello, World!" << std::endl;
    return 0;
}
