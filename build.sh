echo "Configuring and building g2o ..."

cd dependencies/g2o

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8

cd ../../../

echo "Configuring and building VDO-SLAM ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
