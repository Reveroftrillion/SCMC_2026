#!/usr/bin/env bash
# Run inside the image. All test/cache/report writes go to a disposable tree.
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /workspace/ros1-port/devel/setup.bash
comparison_tmp=$(mktemp -d /tmp/rmt-comparison-XXXXXXXX)
destination="$comparison_tmp/tree"
python3 /workspace/ros1-port/docker/prepare_comparison.py "$destination"
cd "$destination"
set +e
python3 ros1-port/verification/check_preservation.py > preservation.log 2>&1
preservation=$?
python3 ros1-port/verification/run_differential.py > differential.log 2>&1
differential=$?
python3 ros1-port/tools/preflight.py > preflight.log 2>&1
preflight=$?
python3 ros1-port/tools/check_model_equivalence.py > model.log 2>&1
model=$?
python3 -m unittest discover -s /workspace/ros1-port/src/ctrl_ros1/test -v > adapter.log 2>&1
adapter=$?
python3 -m unittest discover -s /workspace/ros1-port/tools -p test_packaging.py -v > packaging.log 2>&1
packaging=$?
set -e
output=/workspace/ros1-port/docker/artifacts/comparison-$(date -u +%Y%m%dT%H%M%SZ)
mkdir -p "$output"
cp ./*.log checkout-line-endings.json "$output/"
cp ros1-port/verification/{differential-results,preservation-results,case-outcomes}.json "$output/"
cp ros1-port/docs/model-comparison.json "$output/"
printf '{"preservation":%s,"differential":%s,"preflight":%s,"model":%s,"adapter":%s,"packaging":%s}\n' \
  "$preservation" "$differential" "$preflight" "$model" "$adapter" "$packaging" | tee "$output/exit-codes.json"
echo "Reports: $output"
# The pre-existing model README mismatch must remain a reported failure.
[[ $preservation == 0 && $differential == 0 && $preflight == 0 && $model == 0 && $adapter == 0 && $packaging == 0 ]]
