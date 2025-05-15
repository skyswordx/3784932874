sudo date -s "5/13/2025 17:13"

rm -rf ./build && mkdir -p ./build && cd ./build
cmake .. && make -j4