#!/usr/bin/env bash
set -e

if [ -z "$1" ]; then
    echo "사용법: ./tools/apply_test_map.sh <map_name>"
    echo "예시: ./tools/apply_test_map.sh baseline_success"
    echo "사용 가능한 맵:"
    ls test_maps
    exit 1
fi

MAP_NAME="$1"
MAP_DIR="test_maps/$MAP_NAME"

if [ ! -d "$MAP_DIR" ]; then
    echo "오류: $MAP_DIR 폴더가 없습니다."
    echo "사용 가능한 맵:"
    ls test_maps
    exit 1
fi

if [ ! -f "$MAP_DIR/main.world" ]; then
    echo "오류: $MAP_DIR/main.world 파일이 없습니다."
    exit 1
fi

if [ ! -f "$MAP_DIR/practice_path.csv" ]; then
    echo "오류: $MAP_DIR/practice_path.csv 파일이 없습니다."
    exit 1
fi

cp "$MAP_DIR/main.world" worlds/main.world
cp "$MAP_DIR/practice_path.csv" mission/practice_path.csv

echo "적용 완료: $MAP_NAME"
echo "- worlds/main.world"
echo "- mission/practice_path.csv"
