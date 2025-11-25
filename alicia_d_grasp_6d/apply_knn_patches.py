#!/usr/bin/env python3
"""
生成 knn 扩展在现代 PyTorch 下可用的头文件。

运行方式:
    python apply_knn_patches.py
"""

from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parent
KNN_H = ROOT / "grasp_implement/knn/src/knn.h"
VISION_H = ROOT / "grasp_implement/knn/src/cuda/vision.h"


def write_file(target: Path, content: str) -> None:
    target.write_text(content.rstrip() + "\n", encoding="utf-8")
    print(f"[OK] 写入 {target.relative_to(ROOT)}")


def main() -> None:
    knn_h_body = """#pragma once

#include <vector>

#include <torch/extension.h>

#include "cpu/vision.h"

#ifdef WITH_CUDA
#include <ATen/cuda/CUDAContext.h>
#include "cuda/vision.h"
#endif

inline void check_tensor_dtype(const at::Tensor& tensor, at::ScalarType dtype, const char* name) {
  TORCH_CHECK(
      tensor.scalar_type() == dtype,
      name,
      " 必须是 ",
      at::toString(dtype),
      "，当前为 ",
      at::toString(tensor.scalar_type()));
}

inline void ensure_same_device(const at::Tensor& a, const at::Tensor& b, const char* name_a, const char* name_b) {
  TORCH_CHECK(a.device() == b.device(), name_a, " 与 ", name_b, " 必须在同一设备上");
}

inline void ensure_dim(at::Tensor& tensor, int64_t expected, const char* name) {
  TORCH_CHECK(tensor.dim() == expected, name, " 维度必须是 ", expected, "，当前为 ", tensor.dim());
}

inline int knn(at::Tensor ref, at::Tensor query, at::Tensor idx) {
  ensure_dim(ref, 3, "ref");
  ensure_dim(query, 3, "query");
  ensure_dim(idx, 3, "idx");

  ensure_same_device(ref, query, "ref", "query");
  ensure_same_device(ref, idx, "ref", "idx");

  check_tensor_dtype(ref, at::kFloat, "ref");
  check_tensor_dtype(query, at::kFloat, "query");
  check_tensor_dtype(idx, at::kLong, "idx");

  auto ref_contig = ref.contiguous();
  auto query_contig = query.contiguous();
  auto idx_contig = idx.contiguous();

  const auto batch = ref_contig.size(0);
  const auto dim = ref_contig.size(1);
  const auto ref_nb = ref_contig.size(2);
  const auto query_nb = query_contig.size(2);
  const auto k = idx_contig.size(1);

  TORCH_CHECK(k <= ref_nb, "k 不能超过参考点数量");

  auto ref_ptr = ref_contig.data_ptr<float>();
  auto query_ptr = query_contig.data_ptr<float>();
  auto idx_ptr = idx_contig.data_ptr<long>();

  if (ref_contig.is_cuda()) {
#ifdef WITH_CUDA
    auto options = ref_contig.options().dtype(at::kFloat);
    auto dist_buffer = at::empty({ref_nb * query_nb}, options);
    auto dist_ptr = dist_buffer.data_ptr<float>();

    auto stream = c10::cuda::getCurrentCUDAStream();
    for (int64_t b = 0; b < batch; ++b) {
      knn_device(
          ref_ptr + b * dim * ref_nb,
          ref_nb,
          query_ptr + b * dim * query_nb,
          query_nb,
          dim,
          k,
          dist_ptr,
          idx_ptr + b * k * query_nb,
          stream.stream());
    }

    auto err = cudaGetLastError();
    TORCH_CHECK(err == cudaSuccess, "knn CUDA 运行失败: ", cudaGetErrorString(err));
#else
    TORCH_CHECK(false, "未启用 CUDA，无法在 GPU 上运行");
#endif
  } else {
    std::vector<float> dist_buffer(ref_nb * query_nb);
    std::vector<long> index_buffer(ref_nb);
    auto dist_ptr = dist_buffer.data();
    auto index_ptr = index_buffer.data();

    for (int64_t b = 0; b < batch; ++b) {
      knn_cpu(
          ref_ptr + b * dim * ref_nb,
          ref_nb,
          query_ptr + b * dim * query_nb,
          query_nb,
          dim,
          k,
          dist_ptr,
          idx_ptr + b * k * query_nb,
          index_ptr);
    }
  }

  if (!idx.is_contiguous()) {
    idx.copy_(idx_contig);
  }

  return 1;
}
"""

    vision_h_body = """#pragma once

#include <torch/extension.h>

void knn_device(float* ref_dev,
                int ref_width,
                float* query_dev,
                int query_width,
                int height,
                int k,
                float* dist_dev,
                long* ind_dev,
                cudaStream_t stream);
"""

    write_file(KNN_H, knn_h_body)
    write_file(VISION_H, vision_h_body)


if __name__ == "__main__":
    main()

