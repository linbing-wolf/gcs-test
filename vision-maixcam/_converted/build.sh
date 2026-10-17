#!/bin/bash
# 在 pulsar2 容器内执行：/data 挂载到 /home/linbing/maixcam2_convert
# 用法： docker run --rm -v /home/linbing/maixcam2_convert:/data -w /data pulsar2:6.0 bash build.sh
set -e
echo "=== pulsar2 version ==="
pulsar2 version || true

build() {
  local cfg=$1 tag=$2
  echo
  echo "=== BUILD $tag  ($cfg) ==="
  rm -rf tmp_$tag
  pulsar2 build --target_hardware AX620E \
      --input ./export.onnx \
      --output_dir ./tmp_$tag \
      --config ./config/$cfg
  mkdir -p out
  cp tmp_$tag/compiled.axmodel out/cls11n_$tag.axmodel
  ls -lh out/cls11n_$tag.axmodel
}

build cls11n.npu.json  npu
build cls11n.vnpu.json vnpu

echo
echo "=== 产物 ==="
ls -lh out/
echo "=== 模型 IO ==="
onnx inspect -m -t -n out/cls11n_npu.axmodel 2>&1 | head -40 || true